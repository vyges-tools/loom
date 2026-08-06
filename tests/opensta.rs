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
//! VYGES_STA=/usr/local/bin/sta cargo test --test opensta -- --nocapture
//! ```
//!
//! What it found, on a four-line example, all of it invisible to our own reader:
//!
//! - Net-internal nodes were interned into the name map as NAMES with an escaped colon, instead
//!   of being written `*<netid>:<node>`. OpenSTA could not attach them and reported an arrival
//!   of 1.77 ns where the same parasitics give 7.25 ns.
//! - A net's own node was written as a bare `*<id>`, which is a PORT reference in SPEF.
//! - Coupling capacitors were attached to a node number nothing else referenced, leaving them
//!   hanging off a stub: -5.04 ns of slack against -6.45.
//! - And the reader threw the coupling's node pair away, so re-emission put the capacitor on
//!   the driver pin rather than the wire: -6.60 ns against -6.45.

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

/// Run OpenSTA over the example design with `spef` annotated, returning its whole output.
fn run_sta(sta: &Path, work: &Path, spef: &Path) -> String {
    let script = work.join("run.tcl");
    std::fs::write(
        &script,
        format!(
            "read_liberty {}\n\
             read_verilog {}\n\
             link_design top\n\
             read_sdc {}\n\
             read_spef {}\n\
             report_checks -path_delay max -digits 6\n\
             exit\n",
            ex("cells.lib").display(),
            ex("top.v").display(),
            ex("top.sdc").display(),
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

    // 1. The parasitics as they arrived.
    let original = run_sta(&sta, &work, &ex("top.spef"));
    let want = slack(&original)
        .unwrap_or_else(|| panic!("no timing report from the original SPEF:\n{original}"));

    // 2. The same parasitics through our reader and writer.
    let text = std::fs::read_to_string(ex("top.spef")).expect("top.spef");
    let ours_path = work.join("ours.spef");
    std::fs::write(
        &ours_path,
        vyges_loom::spef::Spef::parse(&text).to_spef(&vyges_loom::spef::WriteOpts::default()),
    )
    .expect("write spef");
    let ours = run_sta(&sta, &work, &ours_path);

    // Errors and warnings first: they name the defect, where a slack difference only shows that
    // there is one. "pin n1 not found" is a node we wrote that the design does not have.
    let complaints: Vec<&str> = ours
        .lines()
        .filter(|l| l.starts_with("Error:") || l.starts_with("Warning:"))
        .collect();
    assert!(
        complaints.is_empty(),
        "OpenSTA objected to the SPEF we wrote:\n  {}\n\nfile:\n{}",
        complaints.join("\n  "),
        std::fs::read_to_string(&ours_path).unwrap_or_default()
    );

    let got = slack(&ours).unwrap_or_else(|| panic!("no timing report from our SPEF:\n{ours}"));
    assert_eq!(
        got,
        want,
        "the same parasitics, rewritten, time differently to an outside reader\n\
         --- what we wrote ---\n{}",
        std::fs::read_to_string(&ours_path).unwrap_or_default()
    );
    println!("OpenSTA agrees: slack {want} from both the original SPEF and ours");
    let _ = std::fs::remove_dir_all(&work);
}
