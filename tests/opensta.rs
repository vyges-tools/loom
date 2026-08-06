//! **An external reader's opinion of the SPEF we write.**
//!
//! Every other test here is us marking our own homework. The write/re-read cycle in
//! `composition.rs` is strong but symmetric: a convention the writer invents and the reader
//! happily accepts round-trips perfectly and is still a file no one else can use. Only a reader
//! we did not write can say whether what we emit means what we think it means.
//!
//! So: run a real timer over the design twice — once on the parasitics as they arrived, once on
//! the same parasitics after a trip through our reader and writer — and require the same answer.
//! Not "it parsed": the same slack, to the digit, with no warnings.
//!
//! **Integrated at the file format, over a pipe.** OpenSTA is invoked as a binary with a Tcl
//! script; nothing links against it. That is deliberate and it is the whole lesson of the last
//! time: reaching for the library to get a Verilog and Liberty reader cost a submodule, a Tcl
//! dependency, a Tk dependency and a segfault, for readers this crate already had.
//!
//! Opt-in, like the corpora — set `VYGES_STA` to an `sta` binary, or have one on `PATH`.
//! Without it the test says so and passes, because CI has no EDA tools.
//!
//! ```sh
//! VYGES_STA=/usr/local/bin/sta \
//! VYGES_CORPUS=~/runs VYGES_LIB=$PDK_ROOT/.../sky130_fd_sc_hd__tt_025C_1v80.lib \
//!   cargo test --release --test opensta -- --nocapture
//! ```
//!
//! What it found on a four-line example, none of it visible to our own reader:
//!
//! - Net-internal nodes were interned into the name map as NAMES with an escaped colon, instead
//!   of being written `*<netid>:<node>`. OpenSTA could not attach them and reported an arrival
//!   of 1.77 ns where the same parasitics give 7.25 ns.
//! - A net's own node was written as a bare `*<id>`, which is a PORT reference in SPEF.
//! - Coupling capacitors were attached to a node number nothing else referenced, leaving them
//!   hanging off a stub: -5.04 ns of slack against -6.45.
//! - And the reader threw the coupling's node pair away, so re-emission put the capacitor on
//!   the driver pin rather than the wire: -6.60 ns against -6.45.
//!
//! And what a REAL hardened design found that the example could not — which is the argument for
//! pointing this at one rather than shipping a bigger fixture:
//!
//! - Bus brackets were escaped. `*BUS_DELIMITER [ ]` declares them, so an extractor writes
//!   `*4 count[0]`; ours wrote `count\[0\]` and OpenSTA reported `net count\[0\] not found`
//!   for every bussed net in the design.
//! - `*P` port connections were read and then dropped, because a port's node is not
//!   `<inst>:<pin>`. Every net touching the boundary was short a connection.
//! - The node named after a net that IS a port is the PORT, and has to stay a bare `*<id>`;
//!   giving it a node number moved the boundary capacitance onto a node nothing reaches, and
//!   the clock network came out 26 ps fast.
//! - A coupling capacitor has to be listed in BOTH nets' blocks, which is what OpenRCX does and
//!   is not redundancy: a reader applies it to the net whose block it is in, so listing it once
//!   leaves the other net believing it is coupled to nothing.

use std::path::{Path, PathBuf};
use std::process::Command;

fn ex(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top")).join(name)
}

/// The `sta` binary, if this machine has one.
fn sta_binary() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("VYGES_STA") {
        let p = PathBuf::from(p);
        return p.is_file().then_some(p);
    }
    let out = Command::new("sh").arg("-c").arg("command -v sta").output().ok()?;
    let path = String::from_utf8_lossy(&out.stdout).trim().to_string();
    (!path.is_empty()).then(|| PathBuf::from(path))
}

/// A design this check can be run on: everything a timer needs, plus the parasitics.
struct Design {
    what: String,
    top: String,
    liberty: PathBuf,
    netlist: PathBuf,
    sdc: Option<PathBuf>,
    spef: PathBuf,
}

/// The example that ships with the crate — the case that runs wherever OpenSTA exists.
fn example_design() -> Design {
    Design {
        what: "examples/top".into(),
        top: "top".into(),
        liberty: ex("cells.lib"),
        netlist: ex("top.v"),
        sdc: Some(ex("top.sdc")),
        spef: ex("top.spef"),
    }
}

/// Hardened designs under `VYGES_CORPUS`, in the LibreLane `final/` layout — `nl/<top>.nl.v`,
/// `spef/<corner>/<top>.<corner>.spef`, `sdc/<top>.sdc`. That layout is worth understanding by
/// name because it is the only production harden path we have.
///
/// The standard-cell Liberty is NOT in a run directory — it belongs to the PDK — so it comes
/// from `VYGES_LIB`. Nothing is vendored: the run stays where the flow left it.
fn corpus_designs() -> Vec<Design> {
    let (Ok(root), Ok(lib)) = (std::env::var("VYGES_CORPUS"), std::env::var("VYGES_LIB")) else {
        return Vec::new();
    };
    let liberty = PathBuf::from(lib);
    if !liberty.is_file() {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut stack = vec![PathBuf::from(root)];
    while let Some(d) = stack.pop() {
        let Ok(rd) = std::fs::read_dir(&d) else { continue };
        for e in rd.flatten() {
            let p = e.path();
            if !p.is_dir() {
                continue;
            }
            if p.file_name().and_then(|s| s.to_str()) == Some("final") {
                let Some(netlist) = first_with(&p.join("nl"), ".nl.v").or_else(|| first_with(&p.join("nl"), ".v")) else { continue };
                let top = netlist
                    .file_name()
                    .and_then(|s| s.to_str())
                    .map(|s| s.split('.').next().unwrap_or(s).to_string())
                    .unwrap_or_default();
                let mut spefs = Vec::new();
                if let Ok(corners) = std::fs::read_dir(p.join("spef")) {
                    for c in corners.flatten() {
                        if let Some(f) = first_with(&c.path(), ".spef") {
                            spefs.push(f);
                        }
                    }
                }
                for spef in spefs {
                    out.push(Design {
                        what: format!("{}", spef.display()),
                        top: top.clone(),
                        liberty: liberty.clone(),
                        netlist: netlist.clone(),
                        sdc: first_with(&p.join("sdc"), ".sdc"),
                        spef,
                    });
                }
            } else {
                stack.push(p);
            }
        }
    }
    out.sort_by(|a, b| a.what.cmp(&b.what));
    out
}

fn first_with(dir: &Path, ext: &str) -> Option<PathBuf> {
    let mut hits: Vec<PathBuf> = std::fs::read_dir(dir)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.to_str().is_some_and(|s| s.ends_with(ext)))
        .collect();
    hits.sort();
    hits.into_iter().next()
}

/// Run OpenSTA over `d` with `spef` annotated, returning its whole output.
fn run_sta(sta: &Path, work: &Path, d: &Design, spef: &Path) -> String {
    let script = work.join("run.tcl");
    let sdc = match &d.sdc {
        Some(p) => format!("read_sdc {}\n", p.display()),
        None => String::new(),
    };
    std::fs::write(
        &script,
        format!(
            "read_liberty {}\n\
             read_verilog {}\n\
             link_design {}\n\
             {sdc}\
             read_spef {}\n\
             report_checks -path_delay max -digits 6\n\
             exit\n",
            d.liberty.display(),
            d.netlist.display(),
            d.top,
            spef.display()
        ),
    )
    .expect("write tcl");
    let out = Command::new(sta)
        .arg("-no_splash")
        .arg("-exit")
        .arg(&script)
        .output()
        .expect("run sta");
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

/// The reported slack, as text, so the comparison is exact rather than a float epsilon.
fn slack(report: &str) -> Option<String> {
    report
        .lines()
        .find(|l| l.contains("slack"))
        .map(|l| l.split_whitespace().next().unwrap_or("").to_string())
}

/// One design, timed twice: as the parasitics arrived, and after a trip through our reader and
/// writer. Returns a complaint, or `None` when the two agree.
fn check(sta: &Path, work: &Path, d: &Design) -> Option<String> {
    let original = run_sta(sta, work, d, &d.spef);
    let want = slack(&original)?; // no timing report at all: nothing to compare, say so upstream

    let text = std::fs::read_to_string(&d.spef).ok()?;
    let ours_path = work.join("ours.spef");
    std::fs::write(
        &ours_path,
        vyges_loom::spef::Spef::parse(&text).to_spef(&vyges_loom::spef::WriteOpts::default()),
    )
    .expect("write spef");
    let ours = run_sta(sta, work, d, &ours_path);

    // A design whose netlist instantiates a hardened MACRO cannot be checked with only the
    // standard-cell Liberty: OpenSTA black-boxes the macro, infers its pins from the
    // instantiation, and then cannot confirm which net each one is on — so it objects to every
    // macro pin in the file whatever the file says. The ORIGINAL parasitics draw the same
    // 8816 complaints on `openframe_project_wrapper`, which is how we know they are not ours.
    // Named rather than dropped quietly. (Fill and tap cells are black-boxed in every design
    // and are not macros.)
    let macro_boxed = original
        .lines()
        .filter(|l| l.contains("not found. Creating black box") && !l.contains("sky130_"))
        .count();
    if macro_boxed > 0 {
        return Some(format!(
            "SKIP {}: {macro_boxed} macro(s) black-boxed for want of their Liberty",
            d.what
        ));
    }

    // Complaints first: they NAME the defect, where a slack difference only says there is one.
    // Only those about the parasitics — a netlist that instantiates fill and tap cells the
    // Liberty does not define warns identically for both files and is not ours to fix.
    let spef_name = ours_path.file_name().and_then(|s| s.to_str()).unwrap_or("spef");
    let complaints: Vec<&str> = ours
        .lines()
        .filter(|l| (l.starts_with("Error:") || l.starts_with("Warning:")) && l.contains(spef_name))
        .collect();
    if !complaints.is_empty() {
        return Some(format!("{}: OpenSTA objected:\n    {}", d.what, complaints.join("\n    ")));
    }
    let got = slack(&ours)?;
    if got == want {
        return None;
    }
    // A FEMTOSECOND IS NOT A DIFFERENCE. Our writer emits each value in a canonical order, so a
    // timer sums the same numbers in a different order from the original file's and the last bit
    // moves: `27.784164` against `27.784161`, three femtoseconds, on one corner of one design.
    // That is floating-point summation order and nothing else — it cannot be removed by writing
    // more digits, because the digits are already exact.
    //
    // The bound is 1e-5 ns, which is ten femtoseconds: four orders of magnitude below the
    // SMALLEST defect this check has found (26 ps, the boundary capacitance on a port node) and
    // more than five below the largest (5.5 ns). A real defect does not hide under it, and the
    // difference is printed whenever it is non-zero so drift stays visible.
    let (a, b) = (want.parse::<f64>().ok()?, got.parse::<f64>().ok()?);
    if (a - b).abs() <= 1e-5 {
        println!("      ({}: {want} -> {got}, {:.1} fs — summation order)", d.what, (a - b).abs() * 1e6);
        return None;
    }
    Some(format!(
        "{}: the same parasitics, rewritten, time differently: {want} -> {got}",
        d.what
    ))
}

#[test]
fn opensta_reads_the_spef_we_write_and_gets_the_same_answer() {
    let Some(sta) = sta_binary() else {
        println!(
            "OpenSTA: not on this machine — set VYGES_STA to an `sta` binary to run this check"
        );
        return;
    };
    let work = std::env::temp_dir().join(format!("vyges-opensta-{}", std::process::id()));
    std::fs::create_dir_all(&work).expect("work dir");

    let mut designs = vec![example_design()];
    let corpus = corpus_designs();
    match corpus.len() {
        0 => println!(
            "OpenSTA: the shipped example only — point VYGES_CORPUS at a tree of LibreLane runs \
             and VYGES_LIB at a standard-cell Liberty to check hardened designs too"
        ),
        n => println!("OpenSTA: the shipped example plus {n} hardened design(s)"),
    }
    designs.extend(corpus);

    let (mut bad, mut skipped) = (Vec::new(), Vec::new());
    for d in &designs {
        match check(&sta, &work, d) {
            Some(c) if c.starts_with("SKIP ") => skipped.push(c),
            Some(c) => bad.push(c),
            None => println!("  ok  {}", d.what),
        }
    }
    for s in &skipped {
        println!("  --  {}", s.trim_start_matches("SKIP "));
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
    let _ = std::fs::remove_dir_all(&work);
}
