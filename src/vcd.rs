//! Minimal VCD reader for **vectored** activity: per-signal transition counts over
//! the dump window, so power can use measured toggle rates instead of an estimate.
//!
//! Scalars and **vectors** are supported: a vector `$var` (`data [3:0]`) expands to
//! per-bit nets (`data[3]…data[0]`) and each change counts the bits that actually flip
//! (Hamming distance), so a bus's per-bit activity is measured, not lumped. Signals are
//! keyed by their **full hierarchical path** (`$scope`/`$upscope`), and a netlist net
//! resolves to one by leaf + optional `scope:` — see [`crate::names`]. Depth reserved: FST.

use std::collections::HashMap;

use crate::names::NetIndex;

#[derive(Debug, Clone, Default)]
pub struct Vcd {
    pub idx: NetIndex,   // full-path toggle counts + leaf index + optional design scope
    pub sim_time_s: f64, // total dumped time in seconds
}

#[derive(Debug)]
pub struct VcdError(pub String);
impl std::fmt::Display for VcdError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "vcd error: {}", self.0)
    }
}
impl std::error::Error for VcdError {}

/// Most windows a single sweep may measure. A backstop against a job that asks for a
/// window far smaller than the dump it is measuring — the failure is memory, and it is
/// better named than hit.
pub const MAX_SWEEP_WINDOWS: usize = 100_000;

/// The shortest span that counts as a duration: 1 fs, the finest unit a VCD `$timescale`
/// can name. Anything thinner is floating-point residue, not simulated time.
const MIN_WINDOW_S: f64 = 1.0e-15;

/// One measured window of a [`VcdSweep`].
///
/// Holds only what differs per window — the bounds, the value-change events seen, and the
/// per-path counts. The name→path index is shared, in [`VcdSweep::decls`].
#[derive(Debug, Clone, Default)]
pub struct SweepWindow {
    pub from_s: f64,     // window start (seconds, absolute dump time)
    pub to_s: f64,       // window end, clamped to the dump's end
    pub sim_time_s: f64, // measured duration = to_s - from_s, clamped to the dump
    pub events: u64,     // value-change records inside the window (an activity indicator)
    pub toggles: std::collections::HashMap<String, u64>, // full path -> transitions
}

/// A **single-pass** sweep: many measurement windows over one parse of one dump.
///
/// Everything except the per-window counts is invariant across windows, so the alternative
/// — re-reading the dump once per window — pays for the same parse N times. This walks the
/// value changes once and routes each transition into the window(s) covering its timestamp.
///
/// [`decls`](VcdSweep::decls) carries the declarations (and the job `scope:`); each window
/// carries only its counts. Use [`window`](VcdSweep::window) to get a toggle-rate view.
#[derive(Debug, Clone, Default)]
pub struct VcdSweep {
    pub decls: NetIndex, // declarations + scope; `toggles` is empty (counts live per window)
    pub windows: Vec<SweepWindow>,
    pub dump_end_s: f64, // last timestamp in the dump
}

impl VcdSweep {
    /// Set the design scope (job `scope:`) used to disambiguate leaf names.
    pub fn with_scope(mut self, scope: Option<String>) -> Self {
        self.decls.scope = scope;
        self
    }

    /// Leaf names declared under more than one scope (ambiguous without a `scope:`).
    pub fn colliding_leaves(&self) -> Vec<String> {
        self.decls.colliding_leaves()
    }

    /// A borrowed toggle-rate view of window `i` — the per-window equivalent of a [`Vcd`].
    pub fn window(&self, i: usize) -> WindowActivity<'_> {
        WindowActivity {
            decls: &self.decls,
            win: &self.windows[i],
        }
    }
}

/// Toggle rates for one window of a [`VcdSweep`], borrowing the shared declaration index.
///
/// Deliberately a borrow, not an owned [`Vcd`]: an owned one would clone every declared path
/// per window, which is the same duplication the single-pass parse exists to avoid.
#[derive(Debug, Clone, Copy)]
pub struct WindowActivity<'a> {
    pub decls: &'a NetIndex,
    pub win: &'a SweepWindow,
}

impl WindowActivity<'_> {
    /// Transitions / second for a netlist net over this window (0 if unresolved,
    /// ambiguous, or the window has no duration). Same resolution rule as [`Vcd::toggle_rate`].
    pub fn toggle_rate(&self, net: &str) -> f64 {
        if self.win.sim_time_s <= 0.0 {
            return 0.0;
        }
        match self.decls.resolve_path(net) {
            Some(p) => self.win.toggles.get(p).copied().unwrap_or(0) as f64 / self.win.sim_time_s,
            None => 0.0,
        }
    }
}

/// A step-held real-valued series to lay alongside a dump: `(time_s, value)` points, each
/// holding until the next one.
#[derive(Debug, Clone, Default)]
pub struct RealSeries {
    pub name: String,
    pub points: Vec<(f64, f64)>,
}

/// What [`annotate_reals`] wrote.
#[derive(Debug, Clone, Copy, Default)]
pub struct AnnotateStats {
    pub declared: usize, // signals added
    pub records: usize,  // value-change records written
    pub injected: usize, // timestamps invented for points between existing ones
    pub tick_s: f64,     // the source dump's timescale
}

/// Write a **copy** of `src` to `dst` carrying `series` as extra real-valued signals under
/// their own `$scope`, so an analysis result can be read in a waveform viewer against the
/// stimulus that produced it.
///
/// The source is never modified, and nothing about the original dump changes: declarations are
/// added before `$enddefinitions`, initial values inside the existing `$dumpvars` block, and
/// each later point at its own timestamp. Identifier codes are chosen from those the file does
/// not already use — reusing one would silently rewrite a design signal, which is the one
/// failure a viewer would render as if it were data.
///
/// A point that falls between two of the dump's timestamps gets its own `#t` line rather than
/// being rounded onto the next one: a power curve whose steps drift off their window
/// boundaries is a different curve.
pub fn annotate_reals(
    src: &str,
    dst: &str,
    scope: &str,
    series: &[RealSeries],
) -> Result<AnnotateStats, VcdError> {
    use std::io::{BufRead, BufWriter, Write};

    if series.is_empty() {
        return Err(VcdError("no series to annotate".into()));
    }
    let f = std::fs::File::open(src).map_err(|e| VcdError(format!("{src}: {e}")))?;
    let mut lines = std::io::BufReader::new(f).lines();
    let out = std::fs::File::create(dst).map_err(|e| VcdError(format!("{dst}: {e}")))?;
    let mut w = BufWriter::new(out);

    // --- header. Buffered rather than streamed: the identifier codes already in use are not
    // all known until the last `$var`, and the new declarations have to be written before
    // `$enddefinitions`. A header is small; a body may not be, and is streamed below.
    let mut header: Vec<String> = Vec::new();
    let mut used: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut ts_tokens: Vec<String> = Vec::new();
    let mut in_timescale = false;
    let mut end_line: Option<String> = None;
    for line in lines.by_ref() {
        let line = line.map_err(|e| VcdError(format!("{src}: {e}")))?;
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.first() == Some(&"$var") {
            if let Some(sym) = toks.get(3) {
                used.insert((*sym).to_string());
            }
        }
        // `$timescale` may carry its value on its own line or the next.
        for t in &toks {
            if in_timescale {
                if *t == "$end" {
                    in_timescale = false;
                } else {
                    ts_tokens.push((*t).to_string());
                }
            } else if *t == "$timescale" {
                in_timescale = true;
            }
        }
        if toks.contains(&"$enddefinitions") {
            end_line = Some(line);
            break;
        }
        header.push(line);
    }
    let Some(end_line) = end_line else {
        return Err(VcdError(format!("{src}: no $enddefinitions")));
    };
    let tick_s = parse_timescale(&ts_tokens.join(""));
    let syms = free_symbols(&used, series.len());
    if syms.len() < series.len() {
        return Err(VcdError("no free identifier codes left in the dump".into()));
    }
    for h in &header {
        writeln!(w, "{h}").map_err(wr)?;
    }
    writeln!(w, "$scope module {scope} $end").map_err(wr)?;
    for (s, sym) in series.iter().zip(&syms) {
        writeln!(w, "$var real 64 {sym} {} $end", s.name).map_err(wr)?;
    }
    writeln!(w, "$upscope $end").map_err(wr)?;
    writeln!(w, "{end_line}").map_err(wr)?;

    // --- body. Each series is walked in time order alongside the dump's own timestamps.
    let mut pending: Vec<std::iter::Peekable<std::slice::Iter<'_, (f64, f64)>>> =
        series.iter().map(|s| s.points.iter().peekable()).collect();
    let mut stats = AnnotateStats {
        declared: series.len(),
        tick_s,
        ..Default::default()
    };
    // Emit every point due at or before `now` (in ticks). Points landing strictly before the
    // dump's next timestamp get one of their own so a step keeps its boundary.
    let flush = |w: &mut BufWriter<std::fs::File>,
                 pending: &mut Vec<std::iter::Peekable<std::slice::Iter<'_, (f64, f64)>>>,
                 stats: &mut AnnotateStats,
                 now_ticks: f64,
                 at_now: bool|
     -> Result<(), VcdError> {
        loop {
            // The earliest tick any series still owes at or before `now`.
            let mut next: Option<f64> = None;
            for p in pending.iter_mut() {
                if let Some((t, _)) = p.peek() {
                    let tk = (t / tick_s).round();
                    if tk <= now_ticks && next.map(|n| tk < n).unwrap_or(true) {
                        next = Some(tk);
                    }
                }
            }
            let Some(tk) = next else { return Ok(()) };
            if tk < now_ticks || !at_now {
                writeln!(w, "#{}", fmt_tick(tk)).map_err(wr)?;
                stats.injected += 1;
            }
            for (i, p) in pending.iter_mut().enumerate() {
                while let Some((t, v)) = p.peek() {
                    if (t / tick_s).round() != tk {
                        break;
                    }
                    writeln!(w, "r{:.6e} {}", v, syms[i]).map_err(wr)?;
                    stats.records += 1;
                    p.next();
                }
            }
            if tk >= now_ticks {
                return Ok(());
            }
        }
    };

    // Initial values belong inside the dump's own `$dumpvars` block — that is what it is for,
    // and a viewer then shows the series from t=0 rather than from the first later timestamp.
    let mut in_dumpvars = false;
    for line in lines {
        let line = line.map_err(|e| VcdError(format!("{src}: {e}")))?;
        let first = line.split_whitespace().next().unwrap_or("");
        if first == "$dumpvars" {
            in_dumpvars = true;
            writeln!(w, "{line}").map_err(wr)?;
            continue;
        }
        if in_dumpvars && first == "$end" {
            flush(&mut w, &mut pending, &mut stats, 0.0, true)?;
            in_dumpvars = false;
            writeln!(w, "{line}").map_err(wr)?;
            continue;
        }
        if let Some(rest) = first.strip_prefix('#') {
            if let Ok(t) = rest.parse::<f64>() {
                if t.is_finite() {
                    // Anything due strictly before this timestamp (each at its own), then the
                    // timestamp, then anything due exactly at it. Doing the "before" pass
                    // first is what keeps the file's times monotonically increasing.
                    flush(&mut w, &mut pending, &mut stats, t - 1.0, false)?;
                    writeln!(w, "{line}").map_err(wr)?;
                    flush(&mut w, &mut pending, &mut stats, t, true)?;
                    continue;
                }
            }
        }
        writeln!(w, "{line}").map_err(wr)?;
    }
    w.flush().map_err(wr)?;
    Ok(stats)
}

/// A VCD timestamp is a whole number of timescale units — render it as one.
fn fmt_tick(t: f64) -> String {
    format!("{}", t as i64)
}

fn wr(e: std::io::Error) -> VcdError {
    VcdError(e.to_string())
}

/// Identifier codes the dump does not already use. VCD codes are printable ASCII 33..126;
/// two-character codes are used so a file already spending the whole single-character range
/// (a few thousand signals is enough) still has room.
fn free_symbols(used: &std::collections::HashSet<String>, n: usize) -> Vec<String> {
    let mut out = Vec::with_capacity(n);
    for a in 33u8..=126 {
        for b in 33u8..=126 {
            if out.len() == n {
                return out;
            }
            let cand = format!("{}{}", a as char, b as char);
            if !used.contains(&cand) {
                out.push(cand);
            }
        }
    }
    out
}

/// A uniform grid of measurement windows: window `i` spans
/// `[from + i*step, from + i*step + window)`, clipped to `to` when one is given.
///
/// A single window (the `activity_window:` case) is the one-element grid, so both paths
/// share one counting loop and cannot drift apart.
#[derive(Debug, Clone, Copy)]
struct Grid {
    from_s: f64,
    window_s: f64,     // may be infinite (run to end of dump)
    step_s: f64,       // may be infinite (a single window)
    to_s: Option<f64>, // hard end; None grows the grid to the end of the dump
}

impl Grid {
    /// The `activity_window:` case as a one-element grid.
    fn single(window: Option<(f64, Option<f64>)>) -> Grid {
        let (from_s, to) = match window {
            Some((f, t)) => (f, t),
            None => (0.0, None),
        };
        let window_s = to.map(|t| t - from_s).unwrap_or(f64::INFINITY);
        Grid {
            from_s,
            window_s,
            step_s: f64::INFINITY,
            to_s: to,
        }
    }

    /// Window count when the grid is bounded; `None` when it grows with the dump.
    fn bounded_len(&self) -> Option<usize> {
        let to = self.to_s?;
        if !self.step_s.is_finite() {
            return Some(1);
        }
        let span = to - self.from_s;
        if span <= 0.0 {
            return Some(0);
        }
        Some(Self::snap(span / self.step_s).ceil().max(1.0) as usize)
    }

    /// Inclusive index range of the windows covering `t`, or `None` when no window does
    /// (before the sweep, after it, or in a gap when `step > window`).
    fn active(&self, t: f64) -> Option<(usize, usize)> {
        let rel = t - self.from_s;
        if rel < 0.0 {
            return None;
        }
        if let Some(to) = self.to_s {
            if t >= to {
                return None;
            }
        }
        // An infinite step is a one-window grid: window 0 covers everything the bounds allow.
        let (lo, hi) = if !self.step_s.is_finite() {
            (0usize, 0usize)
        } else {
            let hi_f = Self::snap(rel / self.step_s).floor();
            let hi = if hi_f > 0.0 { hi_f as usize } else { 0 };
            // Smallest window still open at `t`: i*step + window > rel.
            let lo_f = Self::snap((rel - self.window_s) / self.step_s).floor() + 1.0;
            let lo = if lo_f.is_finite() && lo_f > 0.0 {
                lo_f as usize
            } else {
                0
            };
            (lo, hi)
        };
        if lo > hi {
            return None; // in a gap between windows
        }
        if let Some(n) = self.bounded_len() {
            if lo >= n {
                return None;
            }
            return Some((lo, hi.min(n - 1)));
        }
        Some((lo, hi))
    }

    /// Snap a window index that is a hair off a whole number back onto it.
    ///
    /// Window boundaries are derived by dividing durations that were themselves parsed from
    /// decimal text, so `1020ns - 1000ns` is not exactly `20ns` and `20ns / 5ns` is not
    /// exactly `4`. Left alone, that produces a spurious fifth window, or drops a transition
    /// sitting exactly on a boundary into the window before it — and the same dump measured
    /// from a different time origin then reports different activity, which is the one thing a
    /// toggle *rate* must never do.
    fn snap(x: f64) -> f64 {
        let r = x.round();
        if (x - r).abs() <= 1e-9 * r.abs().max(1.0) {
            r
        } else {
            x
        }
    }

    /// Bounds of window `i`, clipped to the grid's own end (not yet to the dump's).
    fn bounds(&self, i: usize) -> (f64, f64) {
        // `0 * INFINITY` is NaN, and an infinite step is how a single open-ended window is
        // spelled — so window 0 always starts at `from`, arithmetic-free.
        let start = if i == 0 {
            self.from_s
        } else {
            self.from_s + i as f64 * self.step_s
        };
        let end = start + self.window_s;
        match self.to_s {
            Some(to) => (start.min(to), end.min(to)),
            None => (start, end),
        }
    }
}

impl Vcd {
    /// Transitions / second for a netlist net (0 if unresolved, ambiguous, or
    /// zero-duration sim). Resolution is scope-aware — see [`crate::names::NetIndex`].
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

    /// Number of leaf names declared under more than one scope (collision risk when
    /// no `scope:` is set).
    pub fn collisions(&self) -> usize {
        self.idx.collisions()
    }

    pub fn load(path: &str) -> Result<Vcd, VcdError> {
        Vcd::load_windowed(path, None)
    }

    /// Like [`load`](Vcd::load), but restrict activity to a `[from, to)` time window
    /// (seconds; `to = None` runs to end-of-dump). See [`parse_windowed`](Vcd::parse_windowed).
    pub fn load_windowed(path: &str, window: Option<(f64, Option<f64>)>) -> Result<Vcd, VcdError> {
        let text = std::fs::read_to_string(path).map_err(|e| VcdError(format!("{path}: {e}")))?;
        Vcd::parse_windowed(&text, window)
    }

    /// Load with both a time window and a design scope.
    pub fn load_scoped(
        path: &str,
        window: Option<(f64, Option<f64>)>,
        scope: Option<String>,
    ) -> Result<Vcd, VcdError> {
        Ok(Vcd::load_windowed(path, window)?.with_scope(scope))
    }

    pub fn parse(text: &str) -> Result<Vcd, VcdError> {
        Vcd::parse_windowed(text, None)
    }

    /// Parse a VCD into per-net transition counts. When `window = Some((from, to))`,
    /// only transitions with `from <= t < to` (seconds) are counted and `sim_time_s`
    /// is the window duration (clamped to the dumped span); `to = None` runs to
    /// end-of-dump. All value changes still update signal state, so the first
    /// in-window change is measured against the correct pre-window value. Windowing
    /// excludes reset/boot from the measurement (VCD only — SAIF is already cumulative).
    pub fn parse_windowed(text: &str, window: Option<(f64, Option<f64>)>) -> Result<Vcd, VcdError> {
        let mut sweep = parse_grid(text, Grid::single(window))?;
        let w = sweep.windows.pop().unwrap_or_default();
        let mut idx = sweep.decls;
        idx.toggles = w.toggles;
        Ok(Vcd {
            idx,
            sim_time_s: w.sim_time_s,
        })
    }

    /// Parse a dump **once** into a uniform sweep of measurement windows: window `i` spans
    /// `[from + i*step, from + i*step + window)`, up to `to` (or the end of the dump when
    /// `to` is `None`). `step` defaults to `window` — consecutive, non-overlapping windows;
    /// a smaller `step` overlaps them, a larger one leaves gaps.
    ///
    /// This is the power-over-time input: the numbers are the same as calling
    /// [`parse_windowed`](Vcd::parse_windowed) once per window, at one parse instead of N.
    pub fn parse_sweep(
        text: &str,
        from_s: f64,
        to_s: Option<f64>,
        window_s: f64,
        step_s: Option<f64>,
    ) -> Result<VcdSweep, VcdError> {
        let step_s = step_s.unwrap_or(window_s);
        if window_s <= 0.0 || !window_s.is_finite() {
            return Err(VcdError(
                "sweep window must be a positive, finite duration".into(),
            ));
        }
        if step_s <= 0.0 || !step_s.is_finite() {
            return Err(VcdError(
                "sweep step must be a positive, finite duration".into(),
            ));
        }
        if let Some(to) = to_s {
            if to <= from_s {
                return Err(VcdError("sweep 'to' must be greater than 'from'".into()));
            }
        }
        parse_grid(
            text,
            Grid {
                from_s,
                window_s,
                step_s,
                to_s,
            },
        )
    }

    /// [`parse_sweep`](Vcd::parse_sweep) from a file, with the design scope applied.
    pub fn load_sweep(
        path: &str,
        from_s: f64,
        to_s: Option<f64>,
        window_s: f64,
        step_s: Option<f64>,
        scope: Option<String>,
    ) -> Result<VcdSweep, VcdError> {
        let text = std::fs::read_to_string(path).map_err(|e| VcdError(format!("{path}: {e}")))?;
        Ok(Vcd::parse_sweep(&text, from_s, to_s, window_s, step_s)?.with_scope(scope))
    }
}

/// The one counting loop, over a [`Grid`] of windows.
///
/// Value changes are read once; each transition is credited to every window covering its
/// timestamp (one window in the usual non-overlapping case). All changes update signal
/// state whether or not a window is open, so the first counted change in a window is
/// measured against the correct pre-window value.
fn parse_grid(text: &str, grid: Grid) -> Result<VcdSweep, VcdError> {
    #[derive(Default, Clone)]
    struct Bucket {
        toggles: HashMap<String, u64>,
        events: u64,
    }
    // Credit one transition on `path` to every currently-open window.
    fn bump(buckets: &mut [Bucket], active: Option<(usize, usize)>, path: &str) {
        if let Some((lo, hi)) = active {
            for b in &mut buckets[lo..=hi] {
                *b.toggles.entry(path.to_string()).or_insert(0) += 1;
            }
        }
    }
    // A bounded grid is allocated up front; an open-ended one grows as the dump advances.
    fn grow(buckets: &mut Vec<Bucket>, hi: usize) -> Result<(), VcdError> {
        if hi >= MAX_SWEEP_WINDOWS {
            return Err(VcdError(format!(
                "sweep needs more than {MAX_SWEEP_WINDOWS} windows — use a larger window or step"
            )));
        }
        if hi >= buckets.len() {
            buckets.resize(hi + 1, Bucket::default());
        }
        Ok(())
    }
    // One value-change record seen while these windows were open.
    fn count_event(buckets: &mut [Bucket], active: Option<(usize, usize)>) {
        if let Some((lo, hi)) = active {
            for b in &mut buckets[lo..=hi] {
                b.events += 1;
            }
        }
    }
    let mut buckets: Vec<Bucket> = vec![Bucket::default(); grid.bounded_len().unwrap_or(1)];

    let mut tick_s = 1.0e-9; // default 1ns
                             // ONE CODE, MANY NAMES. A VCD identifier code may be declared for several signals — a
                             // net and its port alias share one code, and dumpers emit that routinely. Keyed by a
                             // single `Sig`, the second `$var` overwrote the first and that signal's activity
                             // vanished: it reported a toggle rate of zero while its alias reported the truth.
    let mut sym2sig: HashMap<String, Vec<Sig>> = HashMap::new();
    let mut last: HashMap<String, char> = HashMap::new(); // scalar full path -> last value
    let mut vprev: HashMap<String, Vec<char>> = HashMap::new(); // sym -> last KNOWN bits ('?' = never seen)
    let mut rprev: HashMap<String, String> = HashMap::new(); // real path -> last value
    let mut idx = NetIndex::default();
    let mut scope_stack: Vec<String> = Vec::new();
    let mut time_ticks: f64 = 0.0;
    let mut active = grid.active(0.0);
    if let Some((_, hi)) = active {
        grow(&mut buckets, hi)?;
    }

    let mut toks = text.split_whitespace().peekable();
    while let Some(tok) = toks.next() {
        match tok {
            "$timescale" => {
                // e.g. "1ns" or "1" then "ns"
                let mut unit = String::new();
                for t in toks.by_ref() {
                    if t == "$end" {
                        break;
                    }
                    unit.push_str(t);
                }
                tick_s = parse_timescale(&unit);
            }
            "$scope" => {
                // $scope <type> <name> $end
                let _ty = toks.next();
                let mut name = toks.next().unwrap_or("").to_string();
                // `$scope module  $end` — an UNNAMED scope, which Verilator writes as the
                // root of a dump. Tokenising on whitespace makes the terminator look like
                // the name, so every path in the file came out under a scope called
                // `$end` and matched nothing — not the netlist, not the same dump in FST
                // form. An unnamed scope contributes no path component.
                if name == "$end" {
                    name.clear();
                } else {
                    for t in toks.by_ref() {
                        if t == "$end" {
                            break;
                        }
                    }
                }
                // Pushed even when empty: `$upscope` pops unconditionally, so skipping the
                // push here would pop the PARENT instead and reparent the rest of the file.
                // The empty component is dropped when the path is joined.
                scope_stack.push(name);
            }
            // `$comment` bodies contain anything at all, including things shaped exactly
            // like a timestamp and a value change. Read as data, a commented-out `#999 1!`
            // became a real transition — activity invented out of prose.
            "$comment" => {
                for t in toks.by_ref() {
                    if t == "$end" {
                        break;
                    }
                }
            }
            "$upscope" => {
                for t in toks.by_ref() {
                    if t == "$end" {
                        break;
                    }
                }
                scope_stack.pop();
            }
            "$var" => {
                // $var <type> <width> <sym> <name> [range] $end
                let ty = toks.next().unwrap_or("").to_string();
                let width: usize = toks.next().and_then(|w| w.parse().ok()).unwrap_or(1);
                let sym = toks.next().unwrap_or("").to_string();
                let name = toks.next().unwrap_or("").to_string();
                // remaining tokens before $end: a `[msb:lsb]` range may appear here
                let mut range: Option<String> = None;
                for t in toks.by_ref() {
                    if t == "$end" {
                        break;
                    }
                    if t.starts_with('[') {
                        range = Some(t.to_string());
                    }
                }
                if !sym.is_empty() && !name.is_empty() {
                    let base = join_scope(&scope_stack, &name);
                    let sig = build_sig(&ty, width, &base, range.as_deref(), &mut idx);
                    sym2sig.entry(sym).or_default().push(sig);
                }
            }
            _ => {
                if let Some(rest) = tok.strip_prefix('#') {
                    // PARSED AS A REAL, NOT AN INTEGER. IEEE 1364 says a timestamp is a
                    // whole number of timescale units, but migen writes `#3.2` and viewers
                    // accept it. Parsed as an integer it fails, and a failed parse left the
                    // clock where it was — so in a dump whose every timestamp is fractional
                    // the time never advanced, the dump measured as zero-length, and every
                    // toggle RATE computed from it came out zero. Silently: nothing about a
                    // rate of zero says the times were unreadable.
                    if let Ok(t) = rest.parse::<f64>() {
                        if t.is_finite() {
                            time_ticks = t;
                            active = grid.active(time_ticks * tick_s);
                            if let Some((_, hi)) = active {
                                grow(&mut buckets, hi)?;
                            }
                        }
                    }
                } else if let Some(first) = tok.chars().next() {
                    match first {
                        '0' | '1' | 'x' | 'X' | 'z' | 'Z' | 'u' | 'U' | 'w' | 'W' | 'h' | 'H'
                        | 'l' | 'L' | '-' => {
                            // Scalar change: <value><sym>.
                            //
                            // `last` holds the last KNOWN value. An `x`/`z` is not a level,
                            // it is the absence of one: it neither counts as a transition
                            // nor replaces what we knew. So 0→x→1 counts ONE toggle (the
                            // net did change), 0→x→0 counts none (it did not), and the
                            // x-flood a `$dumpoff` writes over every signal — which is a
                            // statement about the dumper, not the circuit — counts nothing
                            // at all. Counting each of those as a transition inflated
                            // activity, and therefore dynamic power, without a warning.
                            let sym = &tok[1..];
                            count_event(&mut buckets, active);
                            let Some(v) = level(first) else {
                                continue; // unknown: no transition, no new baseline
                            };
                            for sig in sym2sig.get(sym).map(Vec::as_slice).unwrap_or(&[]) {
                                let Sig::Scalar(full) = sig else { continue };
                                let prev = last.insert(full.clone(), v);
                                if prev.map(|p| p != v).unwrap_or(false) {
                                    bump(&mut buckets, active, full);
                                }
                            }
                        }
                        'b' | 'B' => {
                            // Vector change: b<value> <sym> — count each *bit* that flips.
                            // Per bit, the same rule as a scalar: an `x`/`z` bit is unknown,
                            // so it counts nothing and leaves that bit's last known value
                            // in place.
                            let value = &tok[1..];
                            if let Some(sym) = toks.next() {
                                count_event(&mut buckets, active);
                                for sig in sym2sig.get(sym).map(Vec::as_slice).unwrap_or(&[]) {
                                    // A ONE-BIT SIGNAL IS OFTEN DUMPED IN VECTOR FORM. The
                                    // form is the writer's choice, not the signal's: this
                                    // dumper writes `b0 2` for a plain `reg`, and the same
                                    // signal may appear as `02` elsewhere in the same file.
                                    // Handling only the vector case dropped every such
                                    // signal on the floor — declared, never counted, and
                                    // reported as a net with no activity rather than as
                                    // anything unread. It shares `last` with the scalar
                                    // form so the two spellings agree on one baseline.
                                    let bits = match sig {
                                        Sig::Scalar(full) => {
                                            let Some(v) = value.chars().next_back().and_then(level)
                                            else {
                                                continue;
                                            };
                                            let prev = last.insert(full.clone(), v);
                                            if prev.map(|p| p != v).unwrap_or(false) {
                                                bump(&mut buckets, active, full);
                                            }
                                            continue;
                                        }
                                        Sig::Vector { bits } => bits,
                                    };
                                    let cur = pad_bits(value, bits.len());
                                    // KEYED BY THE NET, NOT THE IDENTIFIER. One identifier
                                    // can name the same vector in several scopes — a port
                                    // carried down a hierarchy shares one symbol — and a
                                    // single last-value slot per symbol means the first
                                    // name consumes the change and updates it, so every
                                    // other name it aliases compares against the value that
                                    // was just written and counts nothing. The scalar path
                                    // keys by net and was right; this one credited the
                                    // vector's activity to whichever scope was declared
                                    // first and reported the rest as dead.
                                    let prev = vprev.entry(bits[0].clone()).or_insert_with(|| {
                                        std::iter::repeat_n('?', bits.len()).collect()
                                    });
                                    for (i, c) in cur.iter().enumerate() {
                                        let Some(c) = level(*c) else { continue };
                                        if i < prev.len() {
                                            let was = prev[i];
                                            if was != '?' && was != c {
                                                bump(&mut buckets, active, &bits[i]);
                                            }
                                            prev[i] = c;
                                        }
                                    }
                                }
                            }
                        }
                        'r' | 'R' => {
                            // Real change: r<value> <sym> — not bit-decomposable, so one
                            // toggle per CHANGE. The initial dump is not a change, and
                            // counting it made every real signal report one transition it
                            // never made — the same off-by-one the scalar path avoids by
                            // comparing against a previous value.
                            let value = tok[1..].to_string();
                            if let Some(sym) = toks.next() {
                                count_event(&mut buckets, active);
                                for sig in sym2sig.get(sym).map(Vec::as_slice).unwrap_or(&[]) {
                                    let Sig::Scalar(full) = sig else { continue };
                                    let changed = rprev
                                        .insert(full.clone(), value.clone())
                                        .map(|p| p != value)
                                        .unwrap_or(false);
                                    if changed {
                                        bump(&mut buckets, active, full);
                                    }
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    let dump_end_s = time_ticks * tick_s;
    // A window is measured over the part of it the dump actually covers: a window
    // running past the end of the dump measured over its nominal length would divide
    // real transitions by time that was never simulated.
    let mut windows: Vec<SweepWindow> = buckets
        .into_iter()
        .enumerate()
        .map(|(i, b)| {
            let (w_from, w_to) = grid.bounds(i);
            let from_s = w_from.clamp(0.0, dump_end_s);
            let to_s = w_to.clamp(from_s, dump_end_s);
            // A span thinner than the finest VCD timescale unit is not a short window, it is
            // the residue of clamping two nearly-equal floats. Left as-is it becomes a
            // divisor: a single transition over 7e-24 s reported 1.5e23 toggles/s, and the
            // power built on it read as terawatts. Below a femtosecond, there is no duration.
            let span = to_s - from_s;
            SweepWindow {
                from_s,
                to_s,
                sim_time_s: if span < MIN_WINDOW_S { 0.0 } else { span },
                events: b.events,
                toggles: b.toggles,
            }
        })
        .collect();
    // The dump's last timestamp opens a window that has no time in it. Its transitions are
    // real and belong to the window that just closed — folding them back keeps the invariant
    // that matters: the windows partition the dump's transitions, so a sweep and a whole-dump
    // read count the same events.
    while windows.len() > 1 && windows.last().is_some_and(|w| w.sim_time_s <= 0.0) {
        let tail = windows.pop().expect("checked");
        let prev = windows.last_mut().expect("len > 1");
        for (path, n) in tail.toggles {
            *prev.toggles.entry(path).or_insert(0) += n;
        }
        prev.events += tail.events;
    }
    Ok(VcdSweep {
        decls: idx,
        windows,
        dump_end_s,
    })
}

fn parse_timescale(s: &str) -> f64 {
    let s = s.trim().to_lowercase();
    let units = [
        ("fs", 1e-15),
        ("ps", 1e-12),
        ("ns", 1e-9),
        ("us", 1e-6),
        ("ms", 1e-3),
        ("s", 1.0),
    ];
    for (suf, scale) in units {
        if let Some(num) = s.strip_suffix(suf) {
            let n: f64 = num.trim().parse().unwrap_or(1.0);
            return n * scale;
        }
    }
    1.0e-9
}

/// How a signal maps to netlist nets: a single scalar net, or the per-bit nets of a
/// vector (`data` with `[3:0]` → `data[3]…data[0]`, indexed left→right). Shared with the
/// FST reader.
pub(crate) enum Sig {
    Scalar(String),
    Vector { bits: Vec<String> },
}

/// Build a [`Sig`] for a `$var` and declare its net(s) in `idx`. Reals and 1-bit
/// signals are scalars; wider signals expand to per-bit nets so a gate-level netlist's
/// per-bit nets (`data[0]`) resolve and each bit's toggles are counted independently.
pub(crate) fn build_sig(
    ty: &str,
    width: usize,
    base: &str,
    range: Option<&str>,
    idx: &mut NetIndex,
) -> Sig {
    if ty.eq_ignore_ascii_case("real") || width <= 1 {
        // A ONE-BIT DECLARATION THAT CARRIES A BIT-SELECT IS ONE BIT OF A VECTOR, and has to
        // keep its index. ModelSim, Quartus and others do not dump a vector as a single wide
        // `$var`; they dump one 1-bit `$var` per bit, each with its own identifier:
        //
        //     $var wire 1 ) r_nxt [2] $end
        //     $var wire 1 * r_nxt [1] $end
        //     $var wire 1 + r_nxt [0] $end
        //
        // Dropping the index named all three `r_nxt`, so every bit of every vector in such a
        // dump collapsed onto ONE net. The counts were not merely summed — the readers keep one
        // last-known value per NAME, so each bit was compared against whichever bit changed
        // last, and the total was neither the vector's activity nor any bit's. It also hid the
        // per-bit nets a gate-level netlist asks for by name. Found against the wellen corpus,
        // where the same dumps in FST form disagreed on a third of the files.
        // A BIT-SELECT AND A ONE-BIT RANGE ARE NOT THE SAME DECLARATION. `[2]` says this $var
        // is one bit OF a wider signal and the index belongs in the name; `[0:0]` says the
        // signal's whole range is one bit, and the signal is called `bus`, not `bus[0]` — which
        // is what a SAIF writer emits for it. The colon is the difference.
        let bit_select = range
            .map(str::trim)
            .filter(|r| !r.contains(':') && !ty.eq_ignore_ascii_case("real"))
            .and_then(|_| parse_range(range))
            .map(|(m, _)| m);
        let name = match bit_select {
            Some(b) => format!("{base}[{b}]"),
            None => base.to_string(),
        };
        idx.declare(&name);
        return Sig::Scalar(name);
    }
    let (msb, lsb) = parse_range(range)
        .filter(|(m, l)| (m - l).unsigned_abs() as usize + 1 == width)
        .unwrap_or((width as i64 - 1, 0));
    let step: i64 = if msb >= lsb { -1 } else { 1 }; // position 0 (leftmost bit) = msb
    let mut bits = Vec::with_capacity(width);
    let mut b = msb;
    for _ in 0..width {
        let full = format!("{base}[{b}]");
        idx.declare(&full);
        bits.push(full);
        b += step;
    }
    Sig::Vector { bits }
}

/// Join a scope stack and a leaf into a full path, dropping unnamed scopes.
///
/// An unnamed scope is a real thing — Verilator writes one as the root of a dump — and it
/// contributes no path component. Keeping it puts a leading separator on every name in the
/// file, which then matches nothing: not the netlist, not the same dump in another format.
pub(crate) fn join_scope(scopes: &[String], leaf: &str) -> String {
    let mut out = String::new();
    // `$end` is dropped along with the empty string: it is a VCD keyword, so no design can
    // declare a scope by that name, and a scope carrying it is an unnamed one that some writer
    // mistook its own terminator for a name. GTKWave's vcd2fst does exactly that, and bakes the
    // literal name `$end` into the converted file — so a Verilator dump and its own conversion
    // described the same hierarchy under two different roots, neither of them the real one.
    for s in scopes
        .iter()
        .map(|s| s.trim())
        .filter(|s| !s.is_empty() && *s != "$end")
    {
        out.push_str(s);
        out.push('.');
    }
    out.push_str(leaf);
    out
}

/// Map an IEEE 1164 nine-state character onto a countable level, or `None` for "no level".
///
/// `H` and `L` are the weak 1 and weak 0 — a pull-up holding a bus high is a driven level and a
/// transition to it is a real transition. `U` (uninitialised), `X`, `W` (weak unknown), `Z` and
/// `-` (don't care) are the absence of a level: they count nothing and do not become the
/// baseline, so `0 -> x -> 0` is no toggle and `0 -> x -> 1` is one.
///
/// Both readers share this so a VHDL dump counts the same either way. Before, the VCD reader
/// only recognised `0/1/x/z` as a scalar change at all and silently dropped every `H`/`L`
/// toggle an nvc or GHDL dump writes, while the FST reader counted `U` and `W` as if they were
/// levels — the same waveform, two different activity figures, neither of them right.
pub(crate) fn level(c: char) -> Option<char> {
    match c.to_ascii_lowercase() {
        '0' | 'l' => Some('0'),
        '1' | 'h' => Some('1'),
        _ => None,
    }
}

/// Parse a `[msb:lsb]` (or single `[bit]`) range token into `(msb, lsb)`.
fn parse_range(range: Option<&str>) -> Option<(i64, i64)> {
    let inner = range?.trim_start_matches('[').trim_end_matches(']');
    match inner.split_once(':') {
        Some((m, l)) => Some((m.trim().parse().ok()?, l.trim().parse().ok()?)),
        None => {
            let b: i64 = inner.trim().parse().ok()?;
            Some((b, b))
        }
    }
}

/// Left-extend a VCD vector value to `width` bits (VCD pads with `0`, or the leading
/// `x`/`z`), returning it MSB-first as chars.
pub(crate) fn pad_bits(value: &str, width: usize) -> Vec<char> {
    let chars: Vec<char> = value.chars().collect();
    if chars.len() >= width {
        return chars[chars.len() - width..].to_vec();
    }
    let fill = match chars.first() {
        Some('x') | Some('X') => 'x',
        Some('z') | Some('Z') => 'z',
        _ => '0',
    };
    let mut out = vec![fill; width - chars.len()];
    out.extend(chars);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const VCD: &str = r#"
$timescale 1ns $end
$scope module top $end
$var wire 1 ! clk $end
$var wire 1 " a $end
$upscope $end
$enddefinitions $end
#0
0!
0"
#5
1!
1"
#10
0!
#15
1!
#20
0!
"#;

    // Two scopes with a colliding leaf `clk`: a top-level one and a nested `dut` one.
    const VCD_HIER: &str = r#"
$timescale 1ns $end
$scope module tb $end
$var wire 1 ! clk $end
$scope module dut $end
$var wire 1 @ clk $end
$upscope $end
$upscope $end
$enddefinitions $end
#0
0!
0@
#5
1!
#10
0!
#15
1@
#20
0@
"#;

    #[test]
    fn counts_transitions_and_time() {
        let v = Vcd::parse(VCD).unwrap();
        assert!((v.sim_time_s - 20.0e-9).abs() < 1e-18);
        // clk: 0->1->0->1->0 = 4 transitions over 20 ns -> 200 MHz toggle rate
        assert_eq!(*v.idx.toggles.get("top.clk").unwrap(), 4);
        assert!((v.toggle_rate("clk") - 2.0e8).abs() < 1.0); // leaf resolves (single scope)
        assert_eq!(*v.idx.toggles.get("top.a").unwrap(), 1);
    }

    #[test]
    fn window_restricts_and_rescales() {
        // [5ns,15ns): clk transitions at t=5 (0->1) and t=10 (1->0); t=15 excluded.
        let v = Vcd::parse_windowed(VCD, Some((5.0e-9, Some(15.0e-9)))).unwrap();
        assert_eq!(*v.idx.toggles.get("top.clk").unwrap(), 2);
        assert!((v.sim_time_s - 10.0e-9).abs() < 1e-18);
        assert!((v.toggle_rate("clk") - 2.0e8).abs() < 1.0);
        // 'a' toggles once, at t=5 -> inside the window
        assert_eq!(*v.idx.toggles.get("top.a").unwrap(), 1);
    }

    #[test]
    fn window_open_ended_runs_to_end() {
        // [10ns, end]: clk transitions at 10, 15, 20 all counted (no upper bound).
        let v = Vcd::parse_windowed(VCD, Some((10.0e-9, None))).unwrap();
        assert_eq!(*v.idx.toggles.get("top.clk").unwrap(), 3);
        assert!((v.sim_time_s - 10.0e-9).abs() < 1e-18); // 20ns dump end - 10ns from
    }

    #[test]
    fn window_outside_dump_is_zero_duration() {
        // Beyond the dump -> zero duration, zero rates, no crash.
        let v = Vcd::parse_windowed(VCD, Some((100.0e-9, Some(200.0e-9)))).unwrap();
        assert!(v.sim_time_s.abs() < 1e-18);
        assert_eq!(v.toggle_rate("clk"), 0.0);
    }

    #[test]
    fn no_window_matches_full_dump() {
        // parse() == parse_windowed(None): unchanged behaviour.
        let full = Vcd::parse(VCD).unwrap();
        let none = Vcd::parse_windowed(VCD, None).unwrap();
        assert_eq!(
            full.idx.toggles.get("top.clk"),
            none.idx.toggles.get("top.clk")
        );
        assert!((full.sim_time_s - none.sim_time_s).abs() < 1e-18);
    }

    #[test]
    fn scope_aware_resolution() {
        let v = Vcd::parse(VCD_HIER).unwrap();
        // tb.clk: 0->1->0 = 2 toggles; dut.clk: 0->1->0 = 2 toggles.
        assert_eq!(*v.idx.toggles.get("tb.clk").unwrap(), 2);
        assert_eq!(*v.idx.toggles.get("tb.dut.clk").unwrap(), 2);
        assert_eq!(v.collisions(), 1);
        // Bare `clk` collides tb vs dut -> unresolved (0), no silent pick.
        assert_eq!(v.toggle_rate("clk"), 0.0);
        // scope: dut -> resolves to tb.dut.clk.
        let scoped = Vcd::parse(VCD_HIER)
            .unwrap()
            .with_scope(Some("dut".to_string()));
        assert!((scoped.toggle_rate("clk") - 1.0e8).abs() < 1.0); // 2 / 20ns
    }

    // A 4-bit vector `data[3:0]` exercised over 10 ns.
    const VCD_VEC: &str = r#"
$timescale 1ns $end
$scope module top $end
$var wire 4 ! data [3:0] $end
$upscope $end
$enddefinitions $end
#0
b0000 !
#5
b0011 !
#10
b0101 !
"#;

    #[test]
    fn vector_counts_per_bit_toggles() {
        let v = Vcd::parse(VCD_VEC).unwrap();
        // 0000 -> 0011 : data[1],data[0] flip.  0011 -> 0101 : data[2],data[1] flip.
        assert_eq!(*v.idx.toggles.get("top.data[0]").unwrap(), 1);
        assert_eq!(*v.idx.toggles.get("top.data[1]").unwrap(), 2);
        assert_eq!(*v.idx.toggles.get("top.data[2]").unwrap(), 1);
        assert_eq!(v.idx.toggles.get("top.data[3]").copied().unwrap_or(0), 0);
        // per-bit net resolves; data[1] = 2 toggles / 10 ns
        assert!((v.toggle_rate("data[1]") - 2.0e8).abs() < 1.0);
        // the old behaviour (one toggle for the whole vector) is gone — bits are independent
        assert_eq!(v.collisions(), 0);
    }

    // A bus that changes *twice within one timestep* — bit 0 goes 0->1->0 at #5.
    // A first-vs-last reader (comparing only the timestep's final value against the
    // previous timestep's) would drop both of those toggles; per-assignment Hamming
    // counting must record them. This is the SNUG-2010 dropped-toggle concern
    // (multiple transitions of a bus within a single time step).
    const VCD_VEC_GLITCH: &str = r#"
$timescale 1ns $end
$scope module top $end
$var wire 4 ! data [3:0] $end
$upscope $end
$enddefinitions $end
#0
b0000 !
#5
b0001 !
b0000 !
#10
b0001 !
"#;

    #[test]
    fn sweep_windows_equal_the_same_windows_parsed_one_at_a_time() {
        // THE PROPERTY THE WHOLE SWEEP RESTS ON. Slicing a dump and measuring each slice
        // separately is the obvious implementation and the slow one; this walks the file once
        // and routes each transition into the window covering it. The two must agree exactly,
        // or the fast path is a different measurement wearing the same name.
        let sweep = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 5.0e-9, None).unwrap();
        assert_eq!(sweep.windows.len(), 4);
        for (i, w) in sweep.windows.iter().enumerate() {
            let from = i as f64 * 5.0e-9;
            let one = Vcd::parse_windowed(VCD, Some((from, Some(from + 5.0e-9)))).unwrap();
            assert_eq!(
                w.toggles.get("top.clk").copied().unwrap_or(0),
                one.idx.toggles.get("top.clk").copied().unwrap_or(0),
                "window {i} disagrees with the same window parsed alone"
            );
            assert!((w.sim_time_s - one.sim_time_s).abs() < 1e-18);
            assert!((sweep.window(i).toggle_rate("clk") - one.toggle_rate("clk")).abs() < 1.0);
        }
        // clk toggles at 5, 10, 15, 20 — one per window after the first, and #20 is the dump
        // end so it lands outside [15,20).
        assert_eq!(
            sweep.windows[0]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            0
        );
        assert_eq!(
            sweep.windows[1]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            1
        );
        assert_eq!(
            sweep.windows[3]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            1
        );
    }

    #[test]
    fn sweep_to_end_of_dump_grows_the_grid_and_partitions_its_transitions() {
        // No `to`: the grid extends to cover the dump, so a caller does not have to know how
        // long the simulation ran to sweep all of it.
        let sweep = Vcd::parse_sweep(VCD, 0.0, None, 10.0e-9, None).unwrap();
        assert!((sweep.dump_end_s - 20.0e-9).abs() < 1e-18);
        // [0,10) and [10,20). The dump's final timestamp opens a third window with no time in
        // it; its transitions are folded back into the window that just closed rather than
        // being reported over a zero duration.
        assert_eq!(sweep.windows.len(), 2);
        assert!((sweep.windows[0].sim_time_s - 10.0e-9).abs() < 1e-18);

        // THE INVARIANT: the windows partition the dump's transitions. A sweep and a
        // whole-dump read must agree on how many times each net moved, or one of them is
        // inventing or losing activity.
        let whole = Vcd::parse(VCD).unwrap();
        for net in ["top.clk", "top.a"] {
            let swept: u64 = sweep
                .windows
                .iter()
                .map(|w| w.toggles.get(net).copied().unwrap_or(0))
                .sum();
            assert_eq!(
                swept,
                whole.idx.toggles.get(net).copied().unwrap_or(0),
                "{net}: the sweep and the whole dump disagree on total transitions"
            );
        }
    }

    #[test]
    fn a_window_thinner_than_a_femtosecond_has_no_duration_and_no_rate() {
        // The bug this pins: clamping a window's bounds to the dump's end can leave a span of
        // ~1e-23 s, and dividing a real transition by that produced 1e23 toggles/s — and, two
        // engines later, a power number in terawatts. Nothing about the output said it was
        // arithmetic rather than a measurement.
        let sweep = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 20.0e-9, None).unwrap();
        for w in &sweep.windows {
            assert!(w.sim_time_s == 0.0 || w.sim_time_s >= 1.0e-15);
        }
        // And the guard holds at the API edge regardless of how a window got there.
        let zero = SweepWindow::default();
        let view = WindowActivity {
            decls: &sweep.decls,
            win: &zero,
        };
        assert_eq!(view.toggle_rate("clk"), 0.0);
    }

    #[test]
    fn overlapping_windows_count_a_transition_in_each() {
        // step < window overlaps deliberately (a smoother curve). A transition inside the
        // overlap belongs to both windows: each is a measurement over its own span.
        let sweep = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 10.0e-9, Some(5.0e-9)).unwrap();
        assert_eq!(sweep.windows.len(), 4);
        // clk changes at #5: inside [0,10) and inside [5,15).
        assert_eq!(
            sweep.windows[0]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            1
        );
        assert_eq!(
            sweep.windows[1]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            2
        ); // #5, #10
    }

    #[test]
    fn a_gap_between_windows_counts_nothing() {
        // step > window samples the dump rather than covering it. Transitions in the gaps are
        // counted nowhere — sampling, not measuring, and the caller asked for it.
        let sweep = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 2.0e-9, Some(10.0e-9)).unwrap();
        assert_eq!(sweep.windows.len(), 2); // [0,2) and [10,12)
        assert_eq!(
            sweep.windows[0]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            0
        );
        assert_eq!(
            sweep.windows[1]
                .toggles
                .get("top.clk")
                .copied()
                .unwrap_or(0),
            1
        ); // #10
    }

    #[test]
    fn events_count_value_changes_per_window() {
        // The activity indicator that makes an empty window legible: a window with no events
        // is a window where nothing happened, not a window that failed to measure.
        let sweep = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 5.0e-9, None).unwrap();
        assert_eq!(sweep.windows[0].events, 2); // #0: 0! 0"
        assert_eq!(sweep.windows[1].events, 2); // #5: 1! 1"
        assert_eq!(sweep.windows[2].events, 1); // #10: 0!
        assert_eq!(sweep.windows[3].events, 1); // #15: 1!
    }

    #[test]
    fn shifting_every_timestamp_does_not_change_the_measurement() {
        // Toggle rates are counts over a duration, so re-anchoring a dump's time origin must
        // not move them. Cheap to assert, and it pins the one assumption a windowed measurement
        // makes about absolute time: that it makes none.
        let shifted: String = VCD
            .lines()
            .map(|l| match l.trim().strip_prefix('#') {
                Some(n) => format!("#{}\n", n.trim().parse::<f64>().unwrap() + 1000.0),
                None => format!("{l}\n"),
            })
            .collect();
        let base = Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 5.0e-9, None).unwrap();
        let moved = Vcd::parse_sweep(&shifted, 1000.0e-9, Some(1020.0e-9), 5.0e-9, None).unwrap();
        assert_eq!(base.windows.len(), moved.windows.len());
        for (i, (b, m)) in base.windows.iter().zip(moved.windows.iter()).enumerate() {
            assert_eq!(
                b.toggles, m.toggles,
                "window {i} changed under a time shift"
            );
            assert!((b.sim_time_s - m.sim_time_s).abs() < 1e-18);
        }
    }

    #[test]
    fn a_sweep_finer_than_the_cap_is_refused_not_attempted() {
        // The failure mode is memory, and a named error beats an allocation storm.
        let r = Vcd::parse_sweep(VCD, 0.0, None, 1.0e-18, None);
        assert!(r.is_err());
        // Zero and negative windows are refused too — neither describes a measurement.
        assert!(Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 0.0, None).is_err());
        assert!(Vcd::parse_sweep(VCD, 0.0, Some(20.0e-9), 5.0e-9, Some(-1.0e-9)).is_err());
        assert!(Vcd::parse_sweep(VCD, 20.0e-9, Some(10.0e-9), 5.0e-9, None).is_err());
    }

    #[test]
    fn sweep_resolution_is_scope_aware_like_the_single_window() {
        // One window past the end of the dump: its span is clamped to the 20 ns simulated, so
        // dut.clk's transitions at #15 and #20 are both inside it.
        let sweep = Vcd::parse_sweep(VCD_HIER, 0.0, Some(25.0e-9), 25.0e-9, None)
            .unwrap()
            .with_scope(Some("dut".to_string()));
        assert_eq!(sweep.windows.len(), 1);
        assert!((sweep.windows[0].sim_time_s - 20.0e-9).abs() < 1e-18);
        assert!((sweep.window(0).toggle_rate("clk") - 1.0e8).abs() < 1.0); // 2 / 20ns
                                                                           // Without a scope the colliding leaf stays unresolved rather than picking one.
        let bare = Vcd::parse_sweep(VCD_HIER, 0.0, Some(25.0e-9), 25.0e-9, None).unwrap();
        assert_eq!(bare.window(0).toggle_rate("clk"), 0.0);
        assert_eq!(bare.colliding_leaves(), vec!["clk".to_string()]);
    }

    // A dump with a `$dumpvars` block and no `#0` — the shape most simulators write, and the
    // one that decides where initial values go.
    const VCD_DUMPVARS: &str = r#"$timescale 1ns $end
$scope module top $end
$var wire 1 ! clk $end
$var wire 1 " a $end
$upscope $end
$enddefinitions $end
$dumpvars
0!
0"
$end
#5
1!
#10
0!
#15
1!
#20
0!
"#;

    fn annotate_to_string(tag: &str, src: &str, series: &[RealSeries]) -> String {
        // Tagged per test: these run in parallel, and a shared path makes one test read the
        // file another is writing — which reads as a bug in the writer rather than in the test.
        let dir = std::env::temp_dir().join(format!("loom-annot-{tag}"));
        std::fs::create_dir_all(&dir).unwrap();
        let (i, o) = (dir.join("in.vcd"), dir.join("out.vcd"));
        std::fs::write(&i, src).unwrap();
        annotate_reals(
            i.to_str().unwrap(),
            o.to_str().unwrap(),
            "power_sweep",
            series,
        )
        .unwrap();
        std::fs::read_to_string(&o).unwrap()
    }

    #[test]
    fn annotated_copy_carries_the_series_and_still_reads_as_the_same_dump() {
        // The point of writing one: the analysis result sits beside the stimulus that produced
        // it. So the copy has to remain the original dump — every signal, every transition —
        // with signals added, never altered.
        let series = [RealSeries {
            name: "power_total_w".into(),
            points: vec![(0.0, 1.0e-6), (10.0e-9, 3.0e-6), (20.0e-9, 2.0e-6)],
        }];
        let out = annotate_to_string("roundtrip", VCD_DUMPVARS, &series);
        let before = Vcd::parse(VCD_DUMPVARS).unwrap();
        let after = Vcd::parse(&out).unwrap();
        for net in ["top.clk", "top.a"] {
            assert_eq!(
                before.idx.toggles.get(net),
                after.idx.toggles.get(net),
                "{net} changed in the annotated copy"
            );
        }
        assert!((before.sim_time_s - after.sim_time_s).abs() < 1e-18);
        // The series is declared under its own scope and its changes are counted as a signal.
        assert!(out.contains("$scope module power_sweep $end"));
        assert!(out.contains("$var real 64"));
        assert_eq!(
            *after
                .idx
                .toggles
                .get("power_sweep.power_total_w")
                .expect("series declared"),
            2,
            "three points, two of them changes after the initial value"
        );
    }

    #[test]
    fn initial_values_go_inside_dumpvars_not_after_it() {
        // A viewer reads `$dumpvars` as the state at t=0. Writing the first point after the
        // block instead leaves the curve undefined until the dump's first timestamp — which on
        // this file is 5 ns in, and on a real one can be most of the reset sequence.
        let series = [RealSeries {
            name: "p".into(),
            points: vec![(0.0, 1.5e-6), (10.0e-9, 2.5e-6)],
        }];
        let out = annotate_to_string("dumpvars", VCD_DUMPVARS, &series);
        let dump_start = out.find("$dumpvars").expect("dumpvars");
        let dump_end = out[dump_start..].find("$end").expect("block end") + dump_start;
        assert!(
            out[dump_start..dump_end].contains("r1.500000e-6"),
            "initial value belongs in the $dumpvars block, got:\n{}",
            &out[dump_start..dump_end]
        );
    }

    #[test]
    fn a_point_between_timestamps_gets_its_own_and_time_never_goes_backwards() {
        // A step that lands between two of the dump's timestamps must keep its boundary: a
        // power curve rounded onto the next timestamp is a different curve. The invariant that
        // makes that safe is monotonic time — a viewer rejects a dump that steps backwards.
        let series = [RealSeries {
            name: "p".into(),
            points: vec![(0.0, 1.0), (7.0e-9, 2.0), (12.0e-9, 3.0)],
        }];
        let out = annotate_to_string("between", VCD_DUMPVARS, &series);
        assert!(out.contains("#7"), "the 7 ns point needs its own timestamp");
        let times: Vec<i64> = out
            .lines()
            .filter_map(|l| l.strip_prefix('#'))
            .filter_map(|t| t.trim().parse().ok())
            .collect();
        assert!(
            times.windows(2).all(|w| w[1] >= w[0]),
            "timestamps must not go backwards: {times:?}"
        );
        assert!(times.contains(&12), "the 12 ns point too");
    }

    #[test]
    fn identifier_codes_never_collide_with_the_dumps_own() {
        // THE ONE FAILURE A VIEWER WOULD RENDER AS DATA. Reusing an existing code does not
        // corrupt the file — it silently rewrites a design signal, so the waveform shows the
        // analysis result drawn on top of a net that never had those values.
        let series: Vec<RealSeries> = (0..3)
            .map(|i| RealSeries {
                name: format!("s{i}"),
                points: vec![(0.0, i as f64)],
            })
            .collect();
        let out = annotate_to_string("codes", VCD_DUMPVARS, &series);
        let code_of = |line: &str| line.split_whitespace().nth(3).unwrap_or("").to_string();
        let mut codes: Vec<String> = out
            .lines()
            .filter(|l| l.starts_with("$var"))
            .map(code_of)
            .collect();
        let n = codes.len();
        codes.sort();
        codes.dedup();
        assert_eq!(codes.len(), n, "two signals share an identifier code");
    }

    #[test]
    fn vector_multi_transition_within_timestep_all_counted() {
        let v = Vcd::parse(VCD_VEC_GLITCH).unwrap();
        // bit0: +1 (0->1) and +1 (1->0) at #5, then +1 (0->1) at #10 = 3 total.
        // A first-vs-last reader would see only 0000->0000 (#5) then 0000->0001 (#10) = 1.
        assert_eq!(*v.idx.toggles.get("top.data[0]").unwrap(), 3);
        assert_eq!(v.idx.toggles.get("top.data[1]").copied().unwrap_or(0), 0);
        assert_eq!(v.idx.toggles.get("top.data[2]").copied().unwrap_or(0), 0);
        assert_eq!(v.idx.toggles.get("top.data[3]").copied().unwrap_or(0), 0);
        // 3 transitions over the 10 ns dump.
        assert!((v.toggle_rate("data[0]") - 3.0e8).abs() < 1.0);
    }
}
