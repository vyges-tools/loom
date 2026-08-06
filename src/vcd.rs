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
        let from_s = window.map(|(f, _)| f).unwrap_or(0.0);
        let to_opt = window.and_then(|(_, t)| t);
        // Does the *current* sim time fall in the counting window? Re-evaluated at each `#t`.
        let in_window = |t: f64| {
            t >= from_s
                && match to_opt {
                    Some(to) => t < to,
                    None => true,
                }
        };

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
        let mut count_now = in_window(0.0);

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
                                count_now = in_window(time_ticks * tick_s);
                            }
                        }
                    } else if let Some(first) = tok.chars().next() {
                        match first {
                            '0' | '1' | 'x' | 'X' | 'z' | 'Z' | 'u' | 'U' | 'w' | 'W' | 'h'
                            | 'H' | 'l' | 'L' | '-' => {
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
                                let Some(v) = level(first) else {
                                    continue; // unknown: no transition, no new baseline
                                };
                                for sig in sym2sig.get(sym).map(Vec::as_slice).unwrap_or(&[]) {
                                    let Sig::Scalar(full) = sig else { continue };
                                    let prev = last.insert(full.clone(), v);
                                    if count_now && prev.map(|p| p != v).unwrap_or(false) {
                                        idx.add_toggles(full, 1);
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
                                                let Some(v) =
                                                    value.chars().next_back().and_then(level)
                                                else {
                                                    continue;
                                                };
                                                let prev = last.insert(full.clone(), v);
                                                if count_now
                                                    && prev.map(|p| p != v).unwrap_or(false)
                                                {
                                                    idx.add_toggles(full, 1);
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
                                        let prev =
                                            vprev.entry(bits[0].clone()).or_insert_with(|| {
                                                std::iter::repeat_n('?', bits.len()).collect()
                                            });
                                        for (i, c) in cur.iter().enumerate() {
                                            let Some(c) = level(*c) else { continue };
                                            if i < prev.len() {
                                                let was = prev[i];
                                                if count_now && was != '?' && was != c {
                                                    idx.add_toggles(&bits[i], 1);
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
                                    for sig in sym2sig.get(sym).map(Vec::as_slice).unwrap_or(&[]) {
                                        let Sig::Scalar(full) = sig else { continue };
                                        let changed = rprev
                                            .insert(full.clone(), value.clone())
                                            .map(|p| p != value)
                                            .unwrap_or(false);
                                        if count_now && changed {
                                            idx.add_toggles(full, 1);
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

        let last_time_s = time_ticks * tick_s;
        let sim_time_s = match window {
            None => last_time_s,
            Some((f, t)) => {
                let eff_from = f.clamp(0.0, last_time_s);
                let eff_to = t.unwrap_or(last_time_s).clamp(eff_from, last_time_s);
                eff_to - eff_from
            }
        };
        Ok(Vcd { idx, sim_time_s })
    }
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
    for s in scopes.iter().map(|s| s.trim()).filter(|s| !s.is_empty() && *s != "$end") {
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
