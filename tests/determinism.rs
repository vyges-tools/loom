//! **Parse the same bytes twice, in two processes, and require an identical result.**
//!
//! This is the precondition every other test on the ladder assumes. They all parse a file *once*
//! and compare that answer to something — a fixture, a second reader, a written file, OpenSTA. All
//! of them are only meaningful if the reader is a *function* of its input. Where it is not, they
//! are not checking correctness; they are sampling whichever answer that run happened to produce.
//!
//! # Why a separate process, and not two calls
//!
//! `corpus.rs::parsing_is_deterministic` already parses twice and compares — but in ONE process,
//! and that is the case which cannot fail. The realistic source of non-determinism in Rust is
//! `HashMap`/`HashSet` iteration order, and the hasher is seeded **once per process**: two calls
//! inside one process walk any map in the same order and agree even when the code is wrong.
//!
//! That is not hypothetical. `vyges-ant` shipped in v0.1.29 returning 135, 136 or 137 antenna
//! violations for one unchanged database — nets 32% and 56% over the limit appearing and vanishing
//! — because two iterated `HashMap`s decided which conductor won an `or_insert` that takes the
//! limit from the first region it sees. Its own unit tests, its CI, and a 54-design corpus sweep
//! were green throughout, because each of them ran the tool once.
//!
//! So this test re-executes its own binary. `dump_child` is an ordinary `#[test]`, inert unless
//! the two environment variables are set; the parent runs it twice per file and byte-compares what
//! it wrote. Each child is a fresh process with a fresh hasher seed.
//!
//! # What is compared, and what that choice means
//!
//! The whole parsed structure, via `Debug`. Not a count and not a chosen field: the defect this
//! exists to catch is *ordering*, and a count is exactly what ordering does not change — ant's
//! net totals were stable while its answer was not.
//!
//! Sequences (`Vec`) are therefore compared **in order**: a reader whose output order wanders is
//! the finding. Loom's readers hold their collections in `BTreeMap` and `Vec` already, so this is
//! a guard against regression rather than a hunt — which is the honest description of any test
//! that passes on the day it is written.
//!
//! ```sh
//! cargo test --test determinism -- --nocapture
//! VYGES_CORPUS=~/pdks/sky130A cargo test --test determinism -- --nocapture
//! ```
//!
//! Unset, it runs over the in-repo fixtures alone and still passes trivially — the same trade as
//! `corpus.rs`.

use std::path::{Path, PathBuf};
use std::process::Command;

/// Set on the child: the file to parse.
const IN_ENV: &str = "LOOM_DETERMINISM_IN";
/// Set on the child: where to write the dump. A file, not stdout, so the harness's own
/// "running 1 test" chatter cannot contaminate the comparison.
const OUT_ENV: &str = "LOOM_DETERMINISM_OUT";

/// Files under `VYGES_CORPUS` plus the in-repo fixtures, as in `corpus.rs`.
fn corpus(exts: &[&str]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack: Vec<PathBuf> = Vec::new();
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests");
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
                let skip = matches!(
                    p.file_name().and_then(|s| s.to_str()),
                    Some(".git") | Some("target") | Some("gds") | Some("mag") | Some("doc")
                );
                if !skip {
                    stack.push(p);
                }
            } else if exts.iter().any(|x| p.to_str().is_some_and(|s| s.ends_with(x))) {
                out.push(p);
            }
        }
    }
    out.sort();
    out
}

/// Parse one file according to its extension and render the WHOLE parsed structure.
///
/// `Debug` rather than a hand-written projection: a projection can only compare the fields
/// someone thought to list, and the class of defect here is one that leaves every obvious field
/// intact. A reader that fails to parse contributes the error text — also a fact that must be
/// stable.
fn dump(path: &Path) -> String {
    use vyges_loom::{def::Def, lef::Lef, liberty::Lib, sdc::Sdc, spef::Spef};
    let p = path.to_string_lossy();
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        // Binary (FST) or unreadable: nothing to compare, and saying so is itself stable.
        Err(e) => return format!("unreadable: {e}"),
    };
    if p.ends_with(".lef") || p.ends_with(".tlef") {
        format!("{:?}", Lef::parse(&text))
    } else if p.ends_with(".def") {
        format!("{:?}", Def::parse(&text))
    } else if p.ends_with(".spef") {
        format!("{:?}", Spef::parse(&text))
    } else if p.ends_with(".lib") {
        format!("{:?}", Lib::parse(&text))
    } else if p.ends_with(".sdc") {
        format!("{:?}", Sdc::parse(&text))
    } else if p.ends_with(".v") || p.ends_with(".sv") || p.ends_with(".json") {
        format!("{:?}", vyges_loom::netlist::parse(&text))
    } else if p.ends_with(".vcd") {
        format!("{:?}", vyges_loom::vcd::Vcd::parse(&text))
    } else {
        String::new()
    }
}

/// **Child mode.** Inert as a test; the parent re-executes this binary with the two variables set.
#[test]
fn dump_child() {
    let (Ok(inp), Ok(outp)) = (std::env::var(IN_ENV), std::env::var(OUT_ENV)) else {
        return; // an ordinary test run: nothing to do
    };
    let _ = std::fs::write(outp, dump(Path::new(&inp)));
}

/// Run the child once and return what it wrote.
fn dump_in_fresh_process(file: &Path, tag: usize) -> Option<String> {
    let exe = std::env::current_exe().ok()?;
    let out = std::env::temp_dir().join(format!(
        "loom-determinism-{}-{}-{}.txt",
        std::process::id(),
        tag,
        file.file_name().and_then(|s| s.to_str()).unwrap_or("f")
    ));
    let _ = std::fs::remove_file(&out);
    let st = Command::new(exe)
        .args(["dump_child", "--exact", "--quiet"])
        .env(IN_ENV, file)
        .env(OUT_ENV, &out)
        .output()
        .ok()?;
    if !st.status.success() {
        // A child that panicked is a finding in its own right — surface it rather than skipping.
        return Some(format!(
            "child failed: {}\n{}",
            st.status,
            String::from_utf8_lossy(&st.stderr)
        ));
    }
    let s = std::fs::read_to_string(&out).ok();
    let _ = std::fs::remove_file(&out);
    s
}

/// The test. One file at a time, two fresh processes, byte-compare.
#[test]
fn every_reader_gives_the_same_answer_in_a_fresh_process() {
    let files = corpus(&[
        ".lef", ".tlef", ".def", ".spef", ".lib", ".sdc", ".v", ".sv", ".vcd",
    ]);
    // Bound the run: the corpora are gigabytes and this pays two process spawns per file. The cap
    // is ANNOUNCED rather than silent — an unreported cap reads as full coverage — and it is
    // RAISABLE, because a limit you cannot lift is one nobody ever tests past.
    let cap: usize = std::env::var("LOOM_DETERMINISM_CAP")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(400);
    let capped = files.len().min(cap);
    println!(
        "determinism: {} file(s){}",
        capped,
        if files.len() > cap {
            format!(
                " (capped from {}; raise with LOOM_DETERMINISM_CAP)",
                files.len()
            )
        } else {
            String::new()
        }
    );

    let mut differed = Vec::new();
    let mut compared = 0usize;
    for (i, f) in files.iter().take(capped).enumerate() {
        let (Some(a), Some(b)) = (dump_in_fresh_process(f, i * 2), dump_in_fresh_process(f, i * 2 + 1))
        else {
            continue;
        };
        if a.is_empty() && b.is_empty() {
            continue; // extension we do not read
        }
        compared += 1;
        if a != b {
            let at = a
                .char_indices()
                .zip(b.chars())
                .find(|((_, x), y)| x != y)
                .map(|((i, _), _)| i)
                .unwrap_or_else(|| a.len().min(b.len()));
            differed.push(format!(
                "{}\n     first difference at byte {at}:\n       a: {}\n       b: {}",
                f.display(),
                &a[at.saturating_sub(60)..(at + 60).min(a.len())],
                &b[at.saturating_sub(60)..(at + 60).min(b.len())],
            ));
        }
    }
    println!("determinism: compared {compared} file(s) across two processes each");
    assert!(
        differed.is_empty(),
        "{} file(s) parsed differently in a second process — the reader is not a function of its \
         input, and every other test that reads these files is sampling rather than checking:\n  {}",
        differed.len(),
        differed.join("\n  ")
    );
}
