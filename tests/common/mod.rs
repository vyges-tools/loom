//! Shared by the integration tests: the SPEF write/re-read cycle.
//!
//! Lives here rather than in one test file because both the in-repo fixtures and the opt-in
//! real-design corpus have to be put through exactly the same cycle — if the two drifted, the
//! corpus would be checking something weaker than the fixtures and nobody would notice.

use std::collections::BTreeMap;

use vyges_loom::spef::{Spef, WriteOpts};

/// Two parasitic values are the same value.
///
/// **Relative**, not absolute, and not a fixed number of decimal places: a written value is
/// decimal text, so a sum over ninety coupling capacitors comes back differing in its last
/// digit — 34.787750 against 34.787752, six parts in a hundred million. Comparing formatted
/// strings calls that a defect and buries the real ones under it. A tolerance of 1e-6 is still
/// four orders of magnitude tighter than the smallest thing that can actually go wrong here: a
/// single dropped capacitor moves a net total by percent, not by parts per million.
fn same(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    (a - b).abs() <= 1e-6 * a.abs().max(b.abs()).max(1e-12)
}

/// Everything a SPEF reader extracted about one net, in a form two parses can be compared in.
#[derive(Default)]
struct NetPrint {
    cap: f64,
    res: f64,
    cc: f64,
    ground: Vec<(String, f64)>,
    r: Vec<(String, f64)>,
    pins: Vec<String>,
    coup: Vec<(String, f64)>,
}

fn fingerprint(s: &Spef) -> BTreeMap<String, NetPrint> {
    let mut m = BTreeMap::new();
    for (name, n) in &s.nets {
        let mut p = NetPrint { cap: n.cap_ff, res: n.res_ohm, cc: n.coupling_ff, ..Default::default() };
        p.ground = n.ground.iter().map(|(nd, c)| (nd.clone(), *c)).collect();
        p.r = n.res.iter().map(|(a, b, o)| (format!("{a}-{b}"), *o)).collect();
        p.pins = n.pins.iter().map(|(i, pin, nd)| format!("{i}/{pin}@{nd}")).collect();
        p.coup = n.coupling.iter().map(|(o, c)| (o.clone(), *c)).collect();
        p.ground.sort_by(|a, b| a.0.cmp(&b.0));
        p.r.sort_by(|a, b| a.0.cmp(&b.0));
        p.pins.sort();
        p.coup.sort_by(|a, b| a.0.cmp(&b.0));
        m.insert(name.clone(), p);
    }
    m
}

/// Name the first way two readings of a net differ, or `None` if they agree.
fn net_diff(a: &NetPrint, b: &NetPrint) -> Option<String> {
    for (what, x, y) in
        [("total cap", a.cap, b.cap), ("resistance", a.res, b.res), ("coupling total", a.cc, b.cc)]
    {
        if !same(x, y) {
            return Some(format!("{what} {x} -> {y}"));
        }
    }
    let pairs = [("grounded cap", &a.ground, &b.ground), ("coupling", &a.coup, &b.coup), ("resistor", &a.r, &b.r)];
    for (what, x, y) in pairs {
        if x.len() != y.len() {
            return Some(format!("{what} count {} -> {}", x.len(), y.len()));
        }
        for (i, j) in x.iter().zip(y.iter()) {
            if i.0 != j.0 {
                return Some(format!("{what} node `{}` -> `{}`", i.0, j.0));
            }
            if !same(i.1, j.1) {
                return Some(format!("{what} on `{}`: {} -> {}", i.0, i.1, j.1));
            }
        }
    }
    if a.pins != b.pins {
        return Some(format!("pin hookup {:?} -> {:?}", a.pins, b.pins));
    }
    None
}

/// **The cycle.** Parse → write → parse → write. The two texts must be identical and the two
/// parses must agree about everything, so nothing can be quietly dropped on the way out and
/// nothing invented on the way back in.
///
/// A round trip checked on a handful of hand-picked fields cannot say this: it passes while the
/// writer silently drops every coupling capacitor, because nobody named coupling in the
/// assertion. Comparing the whole structure means the assertion does not have to know in
/// advance which field is going to break.
pub fn survives_the_cycle(text: &str, what: &str) {
    let one = Spef::parse(text);
    let opts = WriteOpts::default();
    let t1 = one.to_spef(&opts);
    let two = Spef::parse(&t1);
    let t2 = two.to_spef(&opts);

    let (fa, fb) = (fingerprint(&one), fingerprint(&two));
    let mut bad: Vec<String> = Vec::new();
    for (name, a) in &fa {
        match fb.get(name) {
            None => bad.push(format!("net `{name}` lost by the cycle")),
            Some(b) => {
                if let Some(d) = net_diff(a, b) {
                    bad.push(format!("net `{name}`: {d}"));
                }
            }
        }
    }
    for name in fb.keys() {
        if !fa.contains_key(name) {
            bad.push(format!("net `{name}` invented by the cycle"));
        }
    }
    // Enough detail to act on, not the whole design back again.
    let shown: Vec<&String> = bad.iter().take(10).collect();
    assert!(
        bad.is_empty(),
        "{what}: the write/re-read cycle changed {} net(s), first {}:\n  {}",
        bad.len(),
        shown.len(),
        shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n  ")
    );

    // A fixed point, not merely a valid file: writing what we just read must reproduce it byte
    // for byte. If it does not, the format has a state the reader normalises and the writer
    // does not, and each further pass through a flow would keep changing the file.
    if t1 != t2 {
        let at = t1
            .lines()
            .zip(t2.lines())
            .position(|(a, b)| a != b)
            .map(|i| format!("line {}:\n    {}\n    {}", i + 1, t1.lines().nth(i).unwrap_or(""), t2.lines().nth(i).unwrap_or("")))
            .unwrap_or_else(|| format!("length {} -> {}", t1.len(), t2.len()));
        panic!("{what}: the writer is not a fixed point on its own output, first difference at {at}");
    }
}
