//! Tiny std-only verbosity control shared by every Vyges binary.
//!
//! The output contract: **data → stdout**, **diagnostics → stderr**. This module
//! only governs the diagnostics (stderr); it never touches stdout, so redirection
//! like `vyges … > data.txt 2> run.log` keeps working. A level gates how much
//! stderr chatter is emitted.
//!
//! Level resolution (highest precedence first):
//!   1. `-v`/`--verbose` and `-q`/`--quiet` flags — repeatable; the net
//!      (verbose − quiet) steps away from the default `info` level.
//!   2. `VYGES_LOG` env var — a level name (`off|error|warn|info|debug|trace`) or a
//!      number `0`–`5`. Inherited by dispatched `vyges-<tool>` children, so it
//!      configures the whole command tree at once.
//!   3. Default: `info`.
//!
//! Shared across the `vyges`, `vyges-catalog`, and `vyges-pdk-store` bins via a
//! `#[path]` module include, so each bin may use only part of it.

use std::sync::atomic::{AtomicU8, Ordering};

/// Diagnostic levels, low value = more severe (mirrors the `log` crate ordering).
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Off = 0,
    Error = 1,
    Warn = 2,
    Info = 3,
    Debug = 4,
    Trace = 5,
}

/// The active level. Set once by [`init`]; read by [`enabled`] / the emit helpers.
static LEVEL: AtomicU8 = AtomicU8::new(Level::Info as u8);

/// Resolve the active level from `-v`/`-q` occurrence counts and `VYGES_LOG`, store
/// it, and return it. Call once at startup, before any diagnostics.
pub fn init(verbose: u8, quiet: u8) -> Level {
    let level = if verbose > 0 || quiet > 0 {
        // Flags win and combine: step from `info` by the net verbosity.
        step(Level::Info as i16 + i16::from(verbose) - i16::from(quiet))
    } else if let Some(l) = std::env::var("VYGES_LOG").ok().and_then(|s| parse(&s)) {
        l
    } else {
        Level::Info
    };
    LEVEL.store(level as u8, Ordering::Relaxed);
    level
}

/// Clamp a numeric level (e.g. `info ± flags`) to a [`Level`].
fn step(n: i16) -> Level {
    match n {
        i if i <= 0 => Level::Off,
        1 => Level::Error,
        2 => Level::Warn,
        3 => Level::Info,
        4 => Level::Debug,
        _ => Level::Trace,
    }
}

/// Parse a `VYGES_LOG` value (case-insensitive name or `0`–`5`). `None` if unknown.
pub fn parse(s: &str) -> Option<Level> {
    match s.trim().to_ascii_lowercase().as_str() {
        "off" | "silent" | "none" | "0" => Some(Level::Off),
        "error" | "1" => Some(Level::Error),
        "warn" | "warning" | "2" => Some(Level::Warn),
        "info" | "3" => Some(Level::Info),
        "debug" | "4" => Some(Level::Debug),
        "trace" | "5" => Some(Level::Trace),
        _ => None,
    }
}

/// Count of a bundled short flag — `short_flag("-qqq", b'q') == 3`, `short_flag("-v",
/// b'v') == 1`, `0` when `s` isn't that repeated short flag. Lets the hand-written
/// arg parsers in the component bins accept `-qqq`/`-vv` like clap does in `vyges`.
pub fn short_flag(s: &str, c: u8) -> u8 {
    let b = s.as_bytes();
    if b.len() >= 2 && b[0] == b'-' && b[1] != b'-' && b[1..].iter().all(|&x| x == c) {
        (b.len() - 1) as u8
    } else {
        0
    }
}

/// Is `level` currently emitted?
pub fn enabled(level: Level) -> bool {
    (level as u8) <= LEVEL.load(Ordering::Relaxed)
}

/// Print one diagnostic line to **stderr** iff `level` is enabled.
pub fn emit(level: Level, msg: &str) {
    if enabled(level) {
        eprintln!("{msg}");
    }
}

pub fn error(msg: &str) {
    emit(Level::Error, msg);
}
pub fn warn(msg: &str) {
    emit(Level::Warn, msg);
}
pub fn info(msg: &str) {
    emit(Level::Info, msg);
}
pub fn debug(msg: &str) {
    emit(Level::Debug, msg);
}
