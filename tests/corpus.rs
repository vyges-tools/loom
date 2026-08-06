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

use vyges_loom::{def::Def, lef::Lef, netlist, saif::Saif, sdc::Sdc, spef::Spef, vcd::Vcd};

/// Files under `VYGES_CORPUS` (recursively) with any of the given extensions, PLUS the
/// in-repo regression fixtures in `tests/data/`.
///
/// The fixtures are unconditional: each pins a construct this reader once got wrong, and they
/// must run in CI where there is no PDK and no flow output. The real-design corpora stay
/// opt-in because they are gigabytes and not ours to ship.
fn corpus(exts: &[&str]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack: Vec<PathBuf> = Vec::new();
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    if fixtures.is_dir() {
        stack.push(fixtures);
    }
    if let Ok(root) = std::env::var("VYGES_CORPUS") {
        stack.push(PathBuf::from(root));
    }
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
    let fixtures = files.iter().filter(|p| p.components().any(|c| c.as_os_str() == "data")).count();
    match (files.len(), fixtures) {
        (0, _) => println!("{what}: nothing to check (set VYGES_CORPUS for real designs)"),
        (n, f) if f == n => println!("{what}: {n} in-repo fixture(s); set VYGES_CORPUS for real designs"),
        (n, f) => println!("{what}: {n} file(s) ({f} in-repo fixture(s) + {} from VYGES_CORPUS)", n - f),
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

/// **Connection conservation on real gate-level Verilog.**
///
/// A netlist reader cannot be checked by "did it parse" — it produces connectivity, and a
/// mangled net name or a dropped connection is a different circuit that still parses. But every
/// `.pin(net)` in the text must become exactly one connection, and that is checkable against any
/// netlist from any tool with no golden answer.
///
/// The ground truth deliberately excludes a `.name(` that follows an identifier character or a
/// `]`, because those occur INSIDE escaped identifiers (`\u_cpu.eoi_unused[31]`) and are not
/// connections. Getting that wrong made this check report 15 phantom losses on a real file.
#[test]
fn every_named_connection_in_a_real_netlist_is_conserved() {
    let files = corpus(&[".v"]);
    announce("Verilog", &files);
    // A path slip must not turn this into a test that checks nothing.
    assert!(
        files.iter().any(|p| p.ends_with("param_override_string.v")),
        "the in-repo netlist fixtures were not found"
    );
    let mut bad = Vec::new();
    for f in &files {
        let Ok(t) = std::fs::read_to_string(f) else { continue };
        // gate-level only: an RTL file is a different language and not this reader's business
        if !t.contains("module ") || !t.contains('(') {
            continue;
        }
        let Ok(n) = netlist::parse(&t) else { continue };
        if n.insts.is_empty() || n.behavioural > 0 {
            continue; // RTL, or an empty wrapper — not this reader's contract
        }
        // `Netlist` returns the TOP module only, so a multi-module file's other modules
        // legitimately contribute connections to the text that are absent from the result.
        if n.modules.len() > 1 {
            continue;
        }
        // Ground truth is counted on the text with COMMENTS REMOVED: a `//` line mentioning
        // `.port_name(wire_name)` is prose, and counting it made this check report losses the
        // parser was right to take.
        // Block comments too: `.out2 (/* unconnected */)` is not a connection, and Yosys' own
        // tests contain exactly that.
        let t = {
            let mut o = String::with_capacity(t.len());
            let b = t.as_bytes();
            let mut i = 0;
            while i < b.len() {
                if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
                    i += 2;
                    while i < b.len() && !(b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/') {
                        if b[i] == b'\n' { o.push('\n'); }
                        i += 1;
                    }
                    i = (i + 2).min(b.len());
                    continue;
                }
                o.push(b[i] as char);
                i += 1;
            }
            o
        };
        let t: String = t
            .lines()
            .map(|l| match l.find("//") {
                Some(i) => &l[..i],
                None => l,
            })
            .collect::<Vec<_>>()
            .join("\n");
        // Byte ranges covered by a `#( … )` PARAMETER list. `\$lcu #(.WIDTH(WIDTH)) impl (…)`
        // has a `.name(` inside it that is a parameter, not a connection — the distinction is
        // positional and cannot be made from the value, which may be an identifier like any net.
        let mut params: Vec<(usize, usize)> = Vec::new();
        {
            let bb = t.as_bytes();
            let mut i = 0;
            while i + 1 < bb.len() {
                if bb[i] == b'#' && bb[i + 1..].iter().position(|c| !c.is_ascii_whitespace()).is_some_and(|k| bb[i + 1 + k] == b'(') {
                    let open = i + 1 + bb[i + 1..].iter().position(|c| !c.is_ascii_whitespace()).unwrap();
                    let mut d = 0;
                    let mut j = open;
                    while j < bb.len() {
                        if bb[j] == b'(' {
                            d += 1;
                        } else if bb[j] == b')' {
                            d -= 1;
                            if d == 0 {
                                break;
                            }
                        }
                        j += 1;
                    }
                    params.push((open, j.min(bb.len())));
                    i = j;
                }
                i += 1;
            }
        }
        let b = t.as_bytes();
        let mut want = 0usize;
        for (i, _) in t.match_indices('.') {
            if params.iter().any(|&(a, z)| i > a && i < z) {
                continue; // a parameter, not a connection
            }
            let r = &t[i + 1..];
            let name: String = r.chars().take_while(|c| c.is_alphanumeric() || *c == '_').collect();
            if name.is_empty() || !r[name.len()..].trim_start().starts_with('(') {
                continue;
            }
            if i > 0 && (b[i - 1].is_ascii_alphanumeric() || b[i - 1] == b'_' || b[i - 1] == b']') {
                continue; // inside an escaped identifier or a bit-select
            }
            // `.pin()` connects nothing, and `.pin(32'h0)` is a constant tie, not a net —
            // neither becomes a connection, by design.
            let val = r[name.len()..].trim_start().trim_start_matches('(').trim_start();
            if val.starts_with(')') || val.starts_with(|c: char| c.is_ascii_digit()) {
                continue;
            }
            want += 1;
        }
        let got: usize = n.insts.iter().map(|i| i.conns.len()).sum();
        // A concatenation is one `.pin(` and several connections, so `got` may legitimately
        // exceed `want`; it must never fall short.
        if got < want {
            bad.push(format!("{}: {want} named connections in the text, {got} parsed", show(f)));
        }
        // and nothing may be recorded under a name that is plainly not a net
        for inst in &n.insts {
            if let Some((p, x)) = inst.conns.iter().find(|(_, x)| {
                x.starts_with('{') || x.ends_with('}') || x.is_empty() || *x == ")"
            }) {
                bad.push(format!("{}: {} pin {p} has a non-net `{x}`", show(f), inst.name));
                break;
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// **Activity must partition.** Toggles counted over the whole dump must equal the toggles of
/// its halves — an internal invariant that needs no reference waveform, and that exercises the
/// window arithmetic the power engine uses when it analyses a slice of a long run.
///
/// It also catches the whole class of "counted something that is not a transition": an `x` from
/// a `$dumpoff`, a `$comment` body, an aliased code credited to one name only. Any of those
/// lands in one half but not in the whole, or the reverse.
#[test]
fn activity_partitions_across_windows() {
    let files = corpus(&[".vcd"]);
    announce("VCD", &files);
    let mut bad = Vec::new();
    for f in &files {
        let Ok(text) = std::fs::read_to_string(f) else { continue };
        // very large dumps are the opt-in corpus's business, not a per-commit check
        if text.len() > 40_000_000 {
            continue;
        }
        let Ok(whole) = Vcd::parse(&text) else {
            bad.push(format!("{}: would not parse", show(f)));
            continue;
        };
        if whole.sim_time_s <= 0.0 {
            continue; // a dump with no time span has nothing to partition
        }
        let mid = whole.sim_time_s / 2.0;
        let (Ok(a), Ok(b)) = (
            Vcd::parse_windowed(&text, Some((0.0, Some(mid)))),
            Vcd::parse_windowed(&text, Some((mid, None))),
        ) else {
            bad.push(format!("{}: windowed parse failed", show(f)));
            continue;
        };
        // compare on the FULL hierarchical path, so leaf-name ambiguity plays no part
        for (path, n) in &whole.idx.toggles {
            let split = a.idx.toggles.get(path).copied().unwrap_or(0)
                + b.idx.toggles.get(path).copied().unwrap_or(0);
            if *n != split {
                bad.push(format!("{}: {path} whole={n} halves={split}", show(f)));
                break;
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// **The FST reader against an external oracle.**
///
/// Wherever a `.vcd` and a `.fst` sit side by side, they are the same waveform in two formats
/// and the two readers must agree exactly — one a line reader, one a clean-room decoder of a
/// compressed binary layout. Generate the pairs with GTKWave's own converter and the oracle is
/// external to us entirely:
///
/// ```sh
/// for f in $(find "$VYGES_CORPUS" -name '*.vcd'); do vcd2fst "$f" "${f%.vcd}.fst"; done
/// ```
///
/// Neither reader can be tuned to pass this without being right: there is no stored expected
/// value, and a drift in either shows up as a disagreement rather than as a changed number.
#[cfg(feature = "fst")]
#[test]
fn the_fst_reader_agrees_with_the_vcd_reader_on_every_pair() {
    use vyges_loom::fst::Fst;
    let vcds = corpus(&[".vcd"]);
    let pairs: Vec<PathBuf> = vcds
        .into_iter()
        .filter(|p| p.with_extension("fst").is_file())
        .collect();
    announce("VCD/FST pairs", &pairs);
    let mut bad = Vec::new();
    let mut aborted = Vec::new();
    for v in &pairs {
        let f = v.with_extension("fst");
        let (Ok(a), Ok(b)) = (Vcd::load(v.to_str().unwrap()), Fst::load(f.to_str().unwrap())) else {
            // A dump killed mid-run has a header and nothing else — there is no hierarchy to
            // compare against, so it says nothing about whether the readers agree. It is
            // NAMED rather than dropped quietly: if a change to the reader started producing
            // these, the count below is where it would show.
            if fst_is_aborted(&f) {
                aborted.push(show(v));
            } else {
                bad.push(format!("{}: one of the pair would not load", show(v)));
            }
            continue;
        };
        if (a.sim_time_s - b.sim_time_s).abs() > 1e-15 * a.sim_time_s.max(1.0) {
            bad.push(format!("{}: sim time vcd {:e} fst {:e}", show(v), a.sim_time_s, b.sim_time_s));
        }
        let mut shown = 0;
        for (path, n) in &a.idx.toggles {
            let m = b.idx.toggles.get(path).copied();
            if m != Some(*n) {
                bad.push(format!("{}: {path} vcd={n} fst={m:?}", show(v)));
                shown += 1;
                if shown >= 3 {
                    break;
                }
            }
        }
        // A net that never toggles gets no entry from either reader, but they can disagree
        // about WHICH those are — so compare on the union, treating a missing key as zero.
        for (path, m) in &b.idx.toggles {
            if !a.idx.toggles.contains_key(path) && *m > 0 {
                bad.push(format!("{}: {path} vcd=absent fst={m}", show(v)));
                break;
            }
        }
    }
    if !aborted.is_empty() {
        println!("  {} aborted dump(s) skipped: {}", aborted.len(), aborted.join(", "));
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// **Toggle counts known BY CONSTRUCTION.**
///
/// A generator writes VCDs and, because it decides every value change itself, records the
/// expected count as arithmetic rather than by asking any tool. Both readers must reproduce it
/// exactly. Where a `.fst` sits alongside (produced by GTKWave's `vcd2fst`), the binary reader
/// is held to the same number — so the chain is: our arithmetic, an external converter, and two
/// independently written parsers, all agreeing or the test fails.
///
/// This is the one check here that can be made arbitrarily exhaustive: generate more files.
/// The generator deliberately emits x/z excursions and `$dumpoff` windows, which is where four
/// of the defects in these readers lived.
///
/// ```sh
/// python3 gen_vcd.py /tmp/vcdgen 200
/// for f in /tmp/vcdgen/*.vcd; do vcd2fst "$f" "${f%.vcd}.fst"; done
/// VYGES_CORPUS=/tmp/vcdgen cargo test --features fst --test corpus counts_known
/// ```
#[test]
fn counts_known_by_construction_are_reproduced_exactly() {
    let files: Vec<PathBuf> = corpus(&[".vcd"])
        .into_iter()
        .filter(|p| p.with_extension("expected").is_file())
        .collect();
    announce("generated VCD", &files);
    let mut bad = Vec::new();
    for f in &files {
        let Ok(exp_txt) = std::fs::read_to_string(f.with_extension("expected")) else { continue };
        let expected: std::collections::BTreeMap<&str, u64> = exp_txt
            .lines()
            .filter_map(|l| l.split_once('\t'))
            .filter_map(|(k, v)| v.trim().parse().ok().map(|n| (k, n)))
            .collect();
        let Ok(v) = Vcd::load(f.to_str().unwrap()) else {
            bad.push(format!("{}: vcd would not load", show(f)));
            continue;
        };
        for (net, want) in &expected {
            let got = v.idx.toggles.get(*net).copied().unwrap_or(0);
            if got != *want {
                bad.push(format!("{}: VCD {net} want {want} got {got}", show(f)));
            }
        }
        for net in v.idx.toggles.keys() {
            if !expected.contains_key(net.as_str()) {
                bad.push(format!("{}: VCD reports {net}, which the generator never wrote", show(f)));
            }
        }
        #[cfg(feature = "fst")]
        {
            let fp = f.with_extension("fst");
            if fp.is_file() {
                match vyges_loom::fst::Fst::load(fp.to_str().unwrap()) {
                    Err(e) => bad.push(format!("{}: fst would not load: {e}", show(f))),
                    Ok(b) => {
                        for (net, want) in &expected {
                            let got = b.idx.toggles.get(*net).copied().unwrap_or(0);
                            if got != *want {
                                bad.push(format!("{}: FST {net} want {want} got {got}", show(f)));
                            }
                        }
                    }
                }
            }
        }
    }
    let n = bad.len();
    bad.truncate(12);
    assert!(bad.is_empty(), "{n} mismatch(es):\n{}", bad.join("\n"));
}

/// **The SAIF reader against an external oracle.**
///
/// Verilator 5.040 writes both formats from one simulation — `--trace` for the VCD and
/// `--trace-saif` for the SAIF — so the same activity arrives twice, described two completely
/// different ways: a log of every transition, and a cumulative `TC` per net. Our two readers
/// must land on the same number, and neither can be tuned to pass without being right.
///
/// This closes the one gap the activity work had left: SAIF semantics were covered by
/// construction (escaping, hierarchy, timescale) but never checked against another
/// implementation.
///
/// ```sh
/// verilator --binary --timing --trace      -o sim_vcd  tb.sv && ./obj_dir/sim_vcd
/// verilator --binary --timing --trace-saif -o sim_saif tb.sv && ./obj_saif/sim_saif
/// VYGES_CORPUS=<dir with wave.vcd + wave.saif> cargo test --test corpus the_saif_reader
/// ```
#[test]
fn the_saif_reader_agrees_with_the_vcd_reader_on_every_pair() {
    let pairs: Vec<PathBuf> = corpus(&[".vcd"])
        .into_iter()
        .filter(|p| p.with_extension("saif").is_file())
        .collect();
    announce("VCD/SAIF pairs", &pairs);
    let mut bad = Vec::new();
    for v in &pairs {
        let sp = v.with_extension("saif");
        let (Ok(a), Ok(b)) = (Vcd::load(v.to_str().unwrap()), Saif::load(sp.to_str().unwrap()))
        else {
            bad.push(format!("{}: one of the pair would not load", show(v)));
            continue;
        };
        // A SAIF states the run's DURATION; a VCD's span is its last timestamp. They describe
        // the same window and must agree.
        if (a.sim_time_s - b.sim_time_s).abs() > 1e-15 * a.sim_time_s.max(1.0) {
            bad.push(format!("{}: span vcd {:e} saif {:e}", show(v), a.sim_time_s, b.sim_time_s));
        }
        let mut shown = 0;
        for (path, n) in &a.idx.toggles {
            let m = b.idx.toggles.get(path).copied();
            if m != Some(*n) {
                bad.push(format!("{}: {path} vcd={n} saif={m:?}", show(v)));
                shown += 1;
                if shown >= 4 {
                    break;
                }
            }
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

/// Does this FST hold only a header — a dump whose writer was killed before it wrote a
/// hierarchy? Such a file has nothing to compare and is not evidence about the reader.
///
/// The block chain is `type u8` then `length u64` big-endian, the length counting itself;
/// `FST_BL_SKIP` (255) or a zero length is the end of what was written.
fn fst_is_aborted(path: &std::path::Path) -> bool {
    let Ok(d) = std::fs::read(path) else { return false };
    let mut o = 0usize;
    while o + 9 <= d.len() {
        let t = d[o];
        let len = u64::from_be_bytes(d[o + 1..o + 9].try_into().unwrap()) as usize;
        if t == 255 || len == 0 {
            return true; // reached the end marker without ever seeing a hierarchy
        }
        if matches!(t, 4 | 6 | 7) {
            return false; // a hierarchy block: the file is real, so a failure to read is ours
        }
        match o.checked_add(1 + len) {
            Some(n) if n > o && n <= d.len() => o = n,
            _ => return false,
        }
    }
    true
}
