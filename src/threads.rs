//! How many worker threads an engine uses — ONE rule for every engine, so the default, the
//! override and the line that reports the choice are the same everywhere.
//!
//! The default is BALANCED, not "every core": the physical cores (a hyperthread sibling adds
//! little to this kind of work), less one for the machine when there are more than four, and at
//! most [`AUTO_CAP`]. Measured on detailed routing, a run is about a quarter serial — 2.7× at 4
//! threads, 3.3× at 16 — while memory grows with every worker (175 MB at 1, 716 MB at 16); past
//! the cap a thread buys little and costs memory.
//!
//! An engine may use this default only when its results do not depend on the thread count — that
//! is checked per engine (its gate run at 1 and at N), not assumed.
//!
//! Override, in order: the command line (`--threads N`), then `VYGES_THREADS`. `0` means "the
//! default"; `1` is strictly serial.

/// The most threads the default picks.
pub const AUTO_CAP: usize = 16;

/// Where the count came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// the engine's command line
    Flag,
    /// the `VYGES_THREADS` environment variable (or an engine's own, older one)
    Env,
    /// [`pick`]'s balanced default
    Auto,
}

/// The thread count chosen, and what it was chosen from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Choice {
    pub threads: usize,
    /// cores this process may run on (affinity and cgroup limits respected)
    pub logical: usize,
    /// physical cores among them
    pub physical: usize,
    pub source: Source,
}

impl Choice {
    /// One line for a log, and for a run's provenance: `using 16 threads of 88 cores (44
    /// physical; balanced default)`.
    pub fn line(&self) -> String {
        let how = match self.source {
            Source::Flag => "set by --threads",
            Source::Env => "set by the environment",
            Source::Auto => "balanced default; --threads N or VYGES_THREADS to change",
        };
        format!(
            "using {} thread{} of {} cores ({} physical; {how})",
            self.threads,
            if self.threads == 1 { "" } else { "s" },
            self.logical,
            self.physical
        )
    }
}

/// The balanced default for a machine with these cores — see the module notes.
pub fn balanced(logical: usize, physical: usize) -> usize {
    let p = physical.clamp(1, logical.max(1));
    let spare = if p > 4 { p - 1 } else { p };
    spare.clamp(1, AUTO_CAP)
}

/// Choose: `flag` (the command line; `Some(0)` is the default), else `VYGES_THREADS`, else each of
/// `legacy_env` in turn (an engine's older variable), else [`balanced`].
pub fn pick(flag: Option<usize>, legacy_env: &[&str]) -> Choice {
    let logical = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
    let physical = physical_cores().unwrap_or((logical / 2).max(1)).min(logical);
    let env = |k: &str| std::env::var(k).ok().and_then(|v| v.trim().parse::<usize>().ok());
    let (n, source) = match flag {
        Some(n) if n > 0 => (n, Source::Flag),
        _ => match std::iter::once("VYGES_THREADS").chain(legacy_env.iter().copied()).find_map(env).filter(|&n| n > 0) {
            Some(n) => (n, Source::Env),
            None => (balanced(logical, physical), Source::Auto),
        },
    };
    Choice { threads: n, logical, physical, source }
}

/// Read `--threads N` / `-j N` / `--threads=N` out of `args`, removing it; `Some(0)` for an
/// explicit `0` (the default), `None` when absent. An unreadable value is an error, not a guess.
pub fn take_flag(args: &mut Vec<String>) -> Result<Option<usize>, String> {
    let mut found = None;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        let (val, width) = if let Some(v) = a.strip_prefix("--threads=") {
            (Some(v.to_string()), 1)
        } else if a == "--threads" || a == "-j" {
            (Some(args.get(i + 1).cloned().ok_or_else(|| format!("{a} needs a number"))?), 2)
        } else {
            i += 1;
            continue;
        };
        let v = val.unwrap_or_default();
        found = Some(v.trim().parse::<usize>().map_err(|_| format!("{a} {v:?} is not a number"))?);
        args.drain(i..i + width);
    }
    Ok(found)
}

/// Physical cores, where the platform says: Linux's CPU topology, macOS's `hw.physicalcpu`.
fn physical_cores() -> Option<usize> {
    if cfg!(target_os = "linux") {
        // distinct (package, core) pairs among the CPUs this process may run on
        let allowed = linux_allowed_cpus();
        let mut cores = std::collections::BTreeSet::new();
        let dir = std::fs::read_dir("/sys/devices/system/cpu").ok()?;
        for e in dir.flatten() {
            let name = e.file_name().to_string_lossy().to_string();
            let Some(id) = name.strip_prefix("cpu").and_then(|n| n.parse::<usize>().ok()) else { continue };
            if allowed.as_ref().is_some_and(|a| !a.contains(&id)) {
                continue;
            }
            let read = |f: &str| std::fs::read_to_string(e.path().join("topology").join(f)).ok().map(|s| s.trim().to_string());
            if let (Some(pkg), Some(core)) = (read("physical_package_id"), read("core_id")) {
                cores.insert((pkg, core));
            }
        }
        return (!cores.is_empty()).then_some(cores.len());
    }
    if cfg!(target_os = "macos") {
        let out = std::process::Command::new("sysctl").args(["-n", "hw.physicalcpu"]).output().ok()?;
        return String::from_utf8_lossy(&out.stdout).trim().parse().ok();
    }
    None
}

/// The CPUs this process may run on (`Cpus_allowed_list`), when Linux says.
fn linux_allowed_cpus() -> Option<std::collections::BTreeSet<usize>> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let list = status.lines().find_map(|l| l.strip_prefix("Cpus_allowed_list:"))?.trim();
    let mut out = std::collections::BTreeSet::new();
    for part in list.split(',') {
        match part.split_once('-') {
            Some((a, b)) => out.extend(a.trim().parse::<usize>().ok()?..=b.trim().parse::<usize>().ok()?),
            None => {
                out.insert(part.trim().parse().ok()?);
            }
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Physical cores, one spare above four, at most [`AUTO_CAP`], at least one.
    #[test]
    fn the_balanced_default() {
        assert_eq!(balanced(1, 1), 1);
        assert_eq!(balanced(8, 4), 4, "four or fewer: all of them");
        assert_eq!(balanced(16, 8), 7, "one left for the machine");
        assert_eq!(balanced(88, 44), AUTO_CAP);
        assert_eq!(balanced(4, 8), 4, "never more than this process may use");
        assert_eq!(balanced(2, 0), 1);
    }

    #[test]
    fn the_flag_is_read_and_removed() {
        let mut a: Vec<String> = ["run", "--threads", "8", "x", "-j=1"].iter().map(|s| s.to_string()).collect();
        assert_eq!(take_flag(&mut a), Ok(Some(8)));
        assert_eq!(a, ["run", "x", "-j=1"]);
        let mut b: Vec<String> = ["--threads=0"].iter().map(|s| s.to_string()).collect();
        assert_eq!(take_flag(&mut b), Ok(Some(0)));
        assert!(b.is_empty());
        let mut c: Vec<String> = ["-j", "many"].iter().map(|s| s.to_string()).collect();
        assert!(take_flag(&mut c).is_err());
        assert_eq!(take_flag(&mut vec![]), Ok(None));
    }

    #[test]
    fn a_flag_beats_the_default_and_zero_is_the_default() {
        assert_eq!(pick(Some(3), &[]).threads, 3);
        assert_eq!(pick(Some(3), &[]).source, Source::Flag);
        let c = pick(Some(0), &["VYGES_TEST_UNSET_THREADS_VAR"]);
        assert!(c.threads >= 1 && c.threads <= AUTO_CAP);
        assert!(c.physical >= 1 && c.physical <= c.logical);
        assert!(c.line().contains("cores"));
    }
}
