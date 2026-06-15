//! `vyges-loom` — the loom binary (the data-plane front end).
//!
//! Hosts the **common, design-wide commands** that operate on the shared design
//! database: parse the standard formats once and inspect / validate them.
//! Tool-specific verbs (timing, power, extraction, LVS, …) belong to the
//! **engines**, which build on the `vyges_loom` library and, in the two-utility
//! packaging, attach here as `vyges-loom <engine>` subcommands.
//!
//! Conventions mirror the rest of the toolchain: a leading subcommand,
//! `--json` machine output, `-q/-v` verbosity, `-h/--help`, `-V/--version`. The
//! version line bakes the git commit (via `build.rs`) exactly like `vyges` does,
//! so a bug report names the precise build.
//!
//! Output contract: **data → stdout**, **diagnostics → stderr**.

use std::process::ExitCode;
use vyges_loom::{verbosity, Design, COPYRIGHT, GIT_SHA, VERSION};

const BUG_URL: &str = "https://github.com/vyges/community/issues/new?labels=bug";
const FEATURE_URL: &str = "https://github.com/vyges/community/issues/new?labels=enhancement";

/// `vyges-loom <ver> (<sha>)` — same shape as `vyges -V`.
fn version_line() -> String {
    format!("vyges-loom {VERSION} ({GIT_SHA})")
}

fn print_help() {
    println!(
        "{ver}

The shared design-data foundation — the \"loom\" the engines weave on. Parses the
standard formats once into a shared in-memory design database. Common, design-wide
commands live here; tool-specific verbs (timing, power, extraction, …) belong to
the engines (built on the vyges_loom library).

USAGE:
  vyges-loom <command> [files...] [options]

COMMANDS:
  inspect <files...>   parse files into the design DB and summarize
  check   <files...>   parse-validate files; non-zero exit on any parse error
  version              print version
  help                 print this help

FILE TYPES (by extension):
  .v / .sv   netlist       .lib   liberty
  .sdc       constraints   .spef  parasitics

OPTIONS:
  --json        machine-readable output (inspect)
  -q, --quiet   fewer diagnostics (repeatable)
  -v, --verbose more diagnostics (repeatable)
  -h, --help    this help
  -V, --version version

Report a bug:      {bug}
Request a feature: {feat}
{copy}",
        ver = version_line(),
        bug = BUG_URL,
        feat = FEATURE_URL,
        copy = COPYRIGHT,
    );
}

/// Load every file into a fresh design (by extension). Returns the design and any
/// per-file errors (does not stop at the first — loom reports all, no silent skip).
fn load_all(files: &[String]) -> (Design, Vec<String>) {
    let mut d = Design::new();
    let mut errs = Vec::new();
    for f in files {
        match d.load(f) {
            Ok(kind) => verbosity::info(&format!("loaded {kind}: {f}")),
            Err(e) => errs.push(e.to_string()),
        }
    }
    (d, errs)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    // -V/--version short-circuits anywhere.
    if args.iter().any(|a| a == "-V" || a == "--version") {
        println!("{}", version_line());
        return ExitCode::SUCCESS;
    }

    let mut verbose = 0u8;
    let mut quiet = 0u8;
    let mut json = false;
    let mut want_help = false;
    let mut positional: Vec<String> = Vec::new();
    for a in &args {
        match a.as_str() {
            "-h" | "--help" => want_help = true,
            "--json" => json = true,
            "--verbose" => verbose += 1,
            "--quiet" => quiet += 1,
            s => {
                let vc = verbosity::short_flag(s, b'v');
                let qc = verbosity::short_flag(s, b'q');
                if vc > 0 {
                    verbose += vc;
                } else if qc > 0 {
                    quiet += qc;
                } else if s.starts_with('-') {
                    eprintln!("vyges-loom: unknown option '{s}' (try --help)");
                    return ExitCode::from(2);
                } else {
                    positional.push(s.to_string());
                }
            }
        }
    }
    verbosity::init(verbose, quiet);

    if want_help {
        print_help();
        return ExitCode::SUCCESS;
    }

    let (cmd, files) = match positional.split_first() {
        Some((c, rest)) => (c.as_str(), rest),
        None => {
            print_help();
            return ExitCode::SUCCESS;
        }
    };

    match cmd {
        "version" => {
            println!("{}", version_line());
            println!("{COPYRIGHT}");
            ExitCode::SUCCESS
        }
        "help" => {
            print_help();
            ExitCode::SUCCESS
        }
        "inspect" => {
            if files.is_empty() {
                eprintln!("vyges-loom inspect: no files given (try --help)");
                return ExitCode::from(2);
            }
            let (d, errs) = load_all(files);
            for e in &errs {
                verbosity::error(&format!("error: {e}"));
            }
            if json {
                println!("{}", d.to_json());
            } else {
                print!("{}", d.summary());
            }
            if errs.is_empty() { ExitCode::SUCCESS } else { ExitCode::FAILURE }
        }
        "check" => {
            if files.is_empty() {
                eprintln!("vyges-loom check: no files given (try --help)");
                return ExitCode::from(2);
            }
            let (_d, errs) = load_all(files);
            for e in &errs {
                verbosity::error(&format!("error: {e}"));
            }
            if errs.is_empty() {
                println!("PASS — {} file(s) parsed clean", files.len());
                ExitCode::SUCCESS
            } else {
                println!("FAIL — {}/{} file(s) failed to parse", errs.len(), files.len());
                ExitCode::FAILURE
            }
        }
        other => {
            eprintln!("vyges-loom: unknown command '{other}' (try --help)");
            ExitCode::from(2)
        }
    }
}
