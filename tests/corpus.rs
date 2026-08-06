//! Invariant checks over **real** LEF/DEF/SPEF, opt-in via environment.
//!
//! Synthetic fixtures pin the constructs we thought of; a real flow output contains the ones we
//! did not. Point these at a PDK or a LibreLane/OpenROAD run directory:
//!
//! ```sh
//! VYGES_CORPUS=~/rcxcorr/calset cargo test --test corpus -- --nocapture
//! VYGES_CORPUS=$PDK_ROOT/sky130A cargo test --test corpus -- --nocapture
//! ```
//!
//! **Point it at someone else's files too.** Our own corpus is all ASIC/OpenLane, so it cannot
//! contain the vendor forms we do not use — running against the constraint files in the
//! Apache-2.0/MIT `sdcx` crate (`git clone https://github.com/dalance/sdcx`, then
//! `VYGES_CORPUS=sdcx/testcase`) found `create_clock -period "100 MHz"`, a Quartus extension
//! that failed two files outright and would never have appeared in ours. Nothing is vendored:
//! the files stay where they are and the corpus is pointed at them.
//!
//! Unset, every test passes trivially — the repo ships no PDK and CI has none. That is the
//! trade: these cannot run everywhere, so they assert **oracle-free invariants** rather than
//! golden values, and can therefore run over any design from any tool without a stored answer.
//!
//! What they are for: the two SPEF bugs this suite was built after were both wrong on day one
//! rather than regressions, so a golden-file comparison could never have caught either. An
//! invariant can.

use std::path::{Path, PathBuf};

use vyges_loom::{def::Def, lef::Lef, sdc::Sdc, spef::Spef};

/// Files under `VYGES_CORPUS` (recursively) with any of the given extensions.
fn corpus(exts: &[&str]) -> Vec<PathBuf> {
    let Ok(root) = std::env::var("VYGES_CORPUS") else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let mut stack = vec![PathBuf::from(root)];
    while let Some(d) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                // a PDK tree is large; skip the obvious non-source directories
                let skip = matches!(
                    p.file_name().and_then(|s| s.to_str()),
                    Some(".git") | Some("target") | Some("gds") | Some("mag") | Some("doc")
                );
                if !skip {
                    stack.push(p);
                }
            } else if exts
                .iter()
                .any(|x| p.to_str().is_some_and(|s| s.ends_with(x)))
            {
                out.push(p);
            }
        }
    }
    out.sort();
    out
}

fn announce(what: &str, files: &[PathBuf]) {
    if files.is_empty() {
        println!("{what}: no corpus (set VYGES_CORPUS to a PDK or flow run directory) — skipped");
    } else {
        println!("{what}: {} file(s)", files.len());
    }
}

/// A tech LEF that parses but yields no usable width is worse than one that fails: extraction
/// then computes every edge-to-edge gap from centre lines and silently over-couples.
#[test]
fn every_tech_lef_yields_usable_routing_layers() {
    let files = corpus(&[".lef", ".tlef"]);
    announce("LEF", &files);
    let mut bad = Vec::new();
    for f in &files {
        let Ok(l) = Lef::load(f.to_str().unwrap()) else { continue };
        if l.layers.is_empty() {
            continue; // a CELL LEF is macros with no tech layers — legal, and not this test
        }
        // health() firing is the detector doing its job — a real LEF 5.8 reference file in this
        // tree declares `UNITS CAPACITANCE 10` and `RESISTANCE 10000`, which we do not apply.
        // That must be REPORTED, not silently used, and reporting it is a pass.
        if l.health().is_some() {
            continue;
        }
        for name in &l.routing_order {
            let layer = &l.layers[name];
            if layer.width_um <= 0.0 {
                bad.push(format!("{}: routing layer {name} has no WIDTH", show(f)));
            }
            // projections must agree with the record they are projected from
            if l.widths.get(name) != Some(&layer.width_um) {
                bad.push(format!("{}: {name} width projection disagrees", show(f)));
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// Parsing is a pure function of the text: the same bytes must give the same answer. Cheap, and
/// it catches any accidental dependence on iteration order or environment.
#[test]
fn parsing_is_deterministic() {
    for f in corpus(&[".lef", ".tlef"]) {
        let Ok(text) = std::fs::read_to_string(&f) else { continue };
        if let (Ok(a), Ok(b)) = (Lef::parse(&text), Lef::parse(&text)) {
            assert_eq!(a.layers.len(), b.layers.len(), "{}", show(&f));
            assert_eq!(a.widths, b.widths, "{}", show(&f));
        }
    }
    for f in corpus(&[".def"]) {
        let Ok(text) = std::fs::read_to_string(&f) else { continue };
        if let (Ok(a), Ok(b)) = (Def::parse(&text), Def::parse(&text)) {
            assert_eq!(a.nets.len(), b.nets.len(), "{}", show(&f));
        }
    }
}

/// Structural invariants a routed DEF must satisfy whatever tool wrote it.
#[test]
fn every_routed_def_is_structurally_sound() {
    let files = corpus(&[".def"]);
    announce("DEF", &files);
    let mut bad = Vec::new();
    for f in &files {
        let Ok(d) = Def::load(f.to_str().unwrap()) else { continue };
        if d.units_per_um <= 0.0 {
            bad.push(format!("{}: no UNITS DISTANCE MICRONS", show(f)));
            continue;
        }
        for n in &d.nets {
            // A pin is `( instance pin )`. Neither field is ever a bare number — when they are,
            // a coordinate group has been read as a connection, which is how a `+ VPIN` once
            // gave a net three pins named after its own geometry.
            for (inst, pin) in &n.pins {
                if inst.parse::<f64>().is_ok() || pin.parse::<f64>().is_ok() {
                    bad.push(format!("{}: net {} has a numeric pin ({inst} {pin})", show(f), n.name));
                    break;
                }
            }
            // Segments must be finite and axis-parallel-or-point; a NaN here reaches the SPEF.
            for s in &n.segments {
                if !(s.x0.is_finite() && s.y0.is_finite() && s.x1.is_finite() && s.y1.is_finite()) {
                    bad.push(format!("{}: net {} has a non-finite segment", show(f), n.name));
                    break;
                }
            }
        }
        // Power straps carry their own width; a zero-width strap is a via landing, not a run.
        for p in &d.power_nets {
            for s in &p.segs {
                if s.width_dbu < 0.0 {
                    bad.push(format!("{}: power net {} has a negative width", show(f), p.name));
                    break;
                }
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// The SPEF invariants, over whatever the corpus holds — see `Spef::health`.
#[test]
fn every_spef_reads_cleanly() {
    let files = corpus(&[".spef", ".spefok"]);
    announce("SPEF", &files);
    let mut bad = Vec::new();
    for f in &files {
        let Ok(s) = Spef::load(f.to_str().unwrap()) else { continue };
        // `skipped` is the tell: entries inside a section we recognise that we could not read.
        if s.skipped > 0 {
            bad.push(format!("{}: {} unreadable entr(ies)", show(f), s.skipped));
        }
        for (name, rc) in &s.nets {
            if !rc.cap_ff.is_finite() || !rc.res_ohm.is_finite() || !rc.coupling_ff.is_finite() {
                bad.push(format!("{}: net {name} has a non-finite value", show(f)));
                break;
            }
            // An aggressor must appear once — the per-aggressor list is what SI switches on.
            let mut seen: Vec<&String> = rc.coupling.iter().map(|(n, _)| n).collect();
            let before = seen.len();
            seen.sort();
            seen.dedup();
            if seen.len() != before {
                bad.push(format!("{}: net {name} lists an aggressor twice", show(f)));
                break;
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// Constraint files must parse, and whatever they lose must be *nameable*. Not modelling a
/// command is defensible; not being able to say which ones were dropped is not.
#[test]
fn every_sdc_parses_and_names_what_it_drops() {
    let files = corpus(&[".sdc"]);
    announce("SDC", &files);
    let mut bad = Vec::new();
    let mut affecting: std::collections::BTreeMap<String, usize> = Default::default();
    for f in &files {
        let Ok(text) = std::fs::read_to_string(f) else { continue };
        match Sdc::parse(&text) {
            // A file that genuinely cannot be read without the flow's environment (an unset
            // `$::env`, an unevaluated `[expr]`) is a legitimate error — provided the message
            // NAMES the offending token, which is the difference between "fix this variable"
            // and a dead end. Generic failures are not acceptable.
            Err(e) => {
                let msg = e.to_string();
                if !msg.contains('`') {
                    bad.push(format!("{}: unactionable error: {msg}", show(f)));
                }
            }
            Ok(s) => {
                for c in s.ignored_affecting_timing() {
                    *affecting.entry(c.to_string()).or_default() += 1;
                }
                // A zero-period clock is the file's own content (generated flows write it),
                // so the requirement is that it be REPORTED rather than passed on quietly.
                let unusable = s.clocks.iter().any(|c| !(c.period.is_finite() && c.period > 0.0));
                if unusable && s.health().is_none() {
                    bad.push(format!("{}: unusable clock period, silently", show(f)));
                }
            }
        }
    }
    if !affecting.is_empty() {
        println!("  unmodelled constraints that can move a slack, by file count:");
        let mut v: Vec<_> = affecting.iter().collect();
        v.sort_by_key(|(k, n)| (std::cmp::Reverse(**n), (*k).clone()));
        for (c, n) in v {
            println!("    {c:<28} {n} file(s)");
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

fn show(p: &Path) -> String {
    p.file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("?")
        .to_string()
}
