//! SDC constraint **linter** — completeness and consistency checks on a design's
//! constraints, independent of the timing run.
//!
//! A correct slack report is worthless if the constraints are wrong: an
//! unconstrained input has *no* path to check, a missing clock leaves registers
//! untimed, a clock on a port the design doesn't have is a typo that silently does
//! nothing. STA tools compute timing on whatever constraints they're given; they
//! rarely tell you the constraints themselves are incomplete. This module does —
//! purely structurally, from the same SDC + netlist (+ Liberty) the timing engine
//! already loads.
//!
//! It is deliberately conservative: it flags only what is structurally certain
//! (an output with no `set_output_delay`, a clock period of zero, two clocks of the
//! same name), so a clean lint means something.
//!
//! Three **depth passes** sit on that floor, sharing one piece of machinery — a local view of
//! the netlist graph:
//!
//!   * **clock-tree tracing** — which SDC clock actually *reaches* each register, rather than
//!     which one was declared. A clock defined on a port that drives nothing leaves every
//!     register untimed while the SDC looks populated, and a design-wide "registers but no
//!     clocks" check cannot see it.
//!   * **exception reachability** — does a `set_false_path` / `set_multicycle_path` name a
//!     path that structurally exists? A typo'd exception does not fail; it silently does
//!     nothing, and the path the author meant stays timed. That is the failure mode worth
//!     closing, because a wrong constraint does not produce a wrong-looking report — it
//!     produces a **clean** one.
//!   * **endpoint coverage** — one number: what fraction of timing endpoints are constrained
//!     at all. An uncovered endpoint is a path nobody is checking.

use std::collections::{BTreeMap, BTreeSet};

use crate::liberty::{Dir, Lib};
use crate::names::instance_of;
use crate::netlist::Netlist;
use crate::sdc::Sdc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warning,
    /// Not a defect — a measurement worth carrying in the report, like endpoint coverage.
    /// Kept distinct so `errors()`/`warnings()` and any CI gate built on them are unaffected
    /// by adding one.
    Info,
}

impl Severity {
    pub fn tag(self) -> &'static str {
        match self {
            Severity::Error => "error",
            Severity::Warning => "warning",
            Severity::Info => "info",
        }
    }
}

/// Does a `get_ports` pattern match a port name? SDC object patterns are globs — `paddr_i[*]`
/// names a whole bus — so an exact-string comparison reports every bus-wide constraint in a
/// real SDC as targeting a port that does not exist. That is a false positive on valid input,
/// which is the one thing a linter cannot afford.
///
/// `*` matches any run of characters, `?` any single one. Anything without a metacharacter
/// compares exactly, so the common case costs nothing.
fn glob_match(pat: &str, name: &str) -> bool {
    if !pat.contains('*') && !pat.contains('?') {
        return pat == name;
    }
    let (p, n): (Vec<char>, Vec<char>) = (pat.chars().collect(), name.chars().collect());
    // Backtracking matcher: `star` remembers the last `*` so a failed tail can retry one
    // character later, which is what makes `a*b*c` work without regex.
    let (mut pi, mut ni) = (0usize, 0usize);
    let (mut star, mut retry) = (usize::MAX, 0usize);
    while ni < n.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == n[ni]) {
            pi += 1;
            ni += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = pi;
            retry = ni;
            pi += 1;
        } else if star != usize::MAX {
            pi = star + 1;
            retry += 1;
            ni = retry;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

/// How many individual ports/registers a per-object finding lists before collapsing into a
/// count. The findings that matter are lost in a list of hundreds, and the count is what a
/// reader acts on anyway.
const PORT_LIST_CAP: usize = 20;

#[derive(Debug, Clone)]
pub struct Finding {
    pub severity: Severity,
    pub code: &'static str, // stable short id, e.g. "clock-period"
    pub message: String,
}

impl Finding {
    fn err(code: &'static str, message: String) -> Finding {
        Finding {
            severity: Severity::Error,
            code,
            message,
        }
    }
    fn warn(code: &'static str, message: String) -> Finding {
        Finding {
            severity: Severity::Warning,
            code,
            message,
        }
    }
}

#[derive(Debug, Default)]
pub struct LintReport {
    pub findings: Vec<Finding>,
}

impl LintReport {
    /// The endpoint-coverage percentage, parsed back out of its finding.
    ///
    /// Kept as a derived accessor rather than a second stored field so there is exactly one
    /// place the number lives and no way for the two to disagree.
    pub fn endpoint_coverage_pct(&self) -> Option<f64> {
        let m = &self
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")?
            .message;
        m.split_whitespace()
            .find_map(|t| t.strip_suffix('%')?.parse().ok())
    }

    pub fn errors(&self) -> usize {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Error)
            .count()
    }
    pub fn warnings(&self) -> usize {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Warning)
            .count()
    }
}

/// Lint the SDC against the design. `lib` lets the linter tell registers from
/// combinational cells (so "registers but no clock" is real, not a guess).
pub fn lint(nl: &Netlist, sdc: &Sdc, lib: &Lib) -> LintReport {
    lint_with_meta(nl, sdc, lib, None)
}

/// As [`lint`], plus the IP's **declared** clock domains from `vyges-metadata.json`.
///
/// Everything else here reasons from the netlist and the SDC, which between them cannot answer
/// "how many clocks *should* this design have had?" — a netlist has no opinion about intent. The
/// declaration supplies that, and it is the only way to catch a flow config that named one
/// `CLOCK_PORT` on a three-clock IP before the block is hardened rather than after.
pub fn lint_with_meta(
    nl: &Netlist,
    sdc: &Sdc,
    lib: &Lib,
    meta: Option<&crate::ipmeta::IpMeta>,
) -> LintReport {
    let mut f = Vec::new();

    let inputs: BTreeSet<&str> = nl.inputs.iter().map(String::as_str).collect();
    let outputs: BTreeSet<&str> = nl.outputs.iter().map(String::as_str).collect();
    // every net name that exists anywhere in the design
    let mut nets: BTreeSet<&str> = inputs.iter().chain(outputs.iter()).copied().collect();
    for inst in &nl.insts {
        for (_, n) in &inst.conns {
            nets.insert(n.as_str());
        }
    }

    // --- the design the constraints name ---------------------------------
    // `current_design` (or `set_current_design`, read as it) naming another design: the file's
    // constraints are still applied to this one, as the current design
    if let Some(d) = sdc.current_design.as_deref().filter(|d| *d != nl.module) {
        f.push(Finding::warn(
            "current-design-not-top",
            format!("`current_design {d}` names a design other than the netlist's top `{}`; the constraints are applied to `{}`", nl.module, nl.module),
        ));
    }

    // --- clocks -------------------------------------------------------------
    let has_registers = nl
        .insts
        .iter()
        .any(|i| lib.cells.get(&i.cell).map(|c| c.is_seq).unwrap_or(false));
    if sdc.clocks.is_empty() && has_registers {
        f.push(Finding::err(
            "no-clock",
            "design has registers but the SDC defines no clocks".into(),
        ));
    }

    let mut by_name: BTreeMap<&str, u32> = BTreeMap::new();
    let mut by_source: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    // Every definition the file wrote, including those a later `create_clock` overwrote or
    // replaced: the parse times only the survivors, but a redefinition is what this reports.
    for c in sdc.clocks.iter().chain(&sdc.superseded_clocks) {
        *by_name.entry(c.name.as_str()).or_default() += 1;
        by_source
            .entry(c.source.as_str())
            .or_default()
            .push(c.name.as_str());
        if c.period <= 0.0 {
            f.push(Finding::err(
                "clock-period",
                format!(
                    "clock `{}` has a non-positive period ({} ns)",
                    c.name, c.period
                ),
            ));
        }
        // a clock whose source is neither a port nor any known net (and not an
        // unresolved inst/pin path) is almost certainly a typo.
        let s = c.source.as_str();
        if !s.contains('/') && !nets.contains(s) {
            f.push(Finding::warn(
                "clock-source",
                format!(
                    "clock `{}` source `{}` is not a port or net in the design",
                    c.name, s
                ),
            ));
        }
    }
    for (name, n) in by_name {
        if n > 1 {
            f.push(Finding::err(
                "dup-clock-name",
                format!("clock `{name}` is defined {n} times"),
            ));
        }
    }
    for (src, names) in by_source {
        if names.len() > 1 {
            f.push(Finding::warn(
                "dup-clock-source",
                format!(
                    "source `{src}` carries {} clocks: {}",
                    names.len(),
                    names.join(", ")
                ),
            ));
        }
    }

    // clock ports are exempt from I/O-delay requirements.
    let clock_ports: BTreeSet<&str> = sdc.clocks.iter().map(|c| c.source.as_str()).collect();

    // --- input / output delay coverage -------------------------------------
    let in_default = sdc.input_delays.iter().any(|d| d.default);
    let out_default = sdc.output_delays.iter().any(|d| d.default);
    let in_ports: BTreeSet<&str> = sdc
        .input_delays
        .iter()
        .flat_map(|d| d.ports.iter().map(String::as_str))
        .collect();
    let out_ports: BTreeSet<&str> = sdc
        .output_delays
        .iter()
        .flat_map(|d| d.ports.iter().map(String::as_str))
        .collect();

    // Listed, then capped. A top-level pad wrapper has hundreds of static configuration
    // outputs that legitimately carry no output delay, and one warning per port buries the
    // findings that matter under a wall the reader scrolls past — which is how a checker gets
    // switched off. Same treatment the per-register check gets.
    let bare_in: Vec<&str> = nl
        .inputs
        .iter()
        .map(String::as_str)
        .filter(|p| {
            !clock_ports.contains(p) && !in_default && !in_ports.iter().any(|q| glob_match(q, p))
        })
        .collect();
    let bare_out: Vec<&str> = nl
        .outputs
        .iter()
        .map(String::as_str)
        .filter(|p| !out_default && !out_ports.iter().any(|q| glob_match(q, p)))
        .collect();
    for (code, kind, cmd, list) in [
        ("unconstrained-input", "input", "set_input_delay", &bare_in),
        (
            "unconstrained-output",
            "output",
            "set_output_delay",
            &bare_out,
        ),
    ] {
        for p in list.iter().take(PORT_LIST_CAP) {
            f.push(Finding::warn(code, format!("{kind} `{p}` has no {cmd}")));
        }
        if list.len() > PORT_LIST_CAP {
            f.push(Finding::warn(
                code,
                format!(
                    "... and {} more {kind}s with no {cmd} ({} in total)",
                    list.len() - PORT_LIST_CAP,
                    list.len()
                ),
            ));
        }
    }

    // an explicit delay targeting a port the design doesn't have
    for p in in_ports
        .iter()
        .filter(|p| !inputs.iter().any(|i| glob_match(p, i)))
    {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_input_delay targets `{p}`, not an input of the design"),
        ));
    }
    for p in out_ports
        .iter()
        .filter(|p| !outputs.iter().any(|o| glob_match(p, o)))
    {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_output_delay targets `{p}`, not an output of the design"),
        ));
    }

    // --- the IP's own declaration, if we were given it -----------------------
    if let Some(m) = meta.filter(|m| !m.is_empty()) {
        let declared_clocks: BTreeSet<&str> =
            sdc.clocks.iter().map(|c| c.source.as_str()).collect();
        for d in &m.domains {
            if !declared_clocks.contains(d.clock.as_str()) {
                f.push(Finding::err(
                    "domain-not-constrained",
                    format!(
                        "the IP declares clock domain `{}` on port `{}`, and the SDC defines no \
                         clock there — every register in that domain is untimed",
                        d.name, d.clock
                    ),
                ));
            }
        }
        // A declared async relationship the SDC does not express means cross-domain paths get
        // timed against an edge relation that does not exist, which produces violations that
        // are not real. Distinct from the above and only a warning: the timing is pessimistic
        // rather than absent.
        if m.async_groups.len() > 1 && sdc.async_groups.len() < 2 {
            f.push(Finding::warn(
                "async-groups-not-declared",
                format!(
                    "the IP declares {} mutually asynchronous clock domains; the SDC has no \
                     set_clock_groups -asynchronous, so paths between them are timed as if \
                     related",
                    m.async_groups.len()
                ),
            ));
        }
    }

    // --- depth passes: clock reach, exception reachability, coverage ---------
    let g = Graph::build(nl, lib);
    let clock_srcs: BTreeMap<&str, &str> = sdc
        .clocks
        .iter()
        .map(|c| (c.source.as_str(), c.name.as_str()))
        .collect();

    // Which clock actually reaches each register. Declared is not reaching.
    let mut unclocked: Vec<&str> = Vec::new();
    let mut clocked = 0usize;
    for (i, inst) in nl.insts.iter().enumerate() {
        let Some(cell) = lib.cells.get(&inst.cell) else {
            continue;
        };
        if !cell.is_seq {
            continue;
        }
        let reached = cell
            .clock_pin
            .as_ref()
            .and_then(|cp| g.net_at(i, cp))
            .and_then(|cn| g.trace_to_source(cn, &clock_srcs));
        match reached {
            Some(_) => clocked += 1,
            None => unclocked.push(inst.name.as_str()),
        }
    }
    // Only worth saying per-register when some clock exists; with none the design-wide error
    // above already says it, and repeating it per register is noise.
    if !sdc.clocks.is_empty() {
        for name in unclocked.iter().take(PORT_LIST_CAP) {
            f.push(Finding::err(
                "register-no-clock-reaches",
                format!("register `{name}`: no SDC clock reaches its clock pin — it is untimed"),
            ));
        }
        if unclocked.len() > PORT_LIST_CAP {
            f.push(Finding::err(
                "register-no-clock-reaches",
                format!(
                    "... and {} more registers no clock reaches ({} in total)",
                    unclocked.len() - PORT_LIST_CAP,
                    unclocked.len()
                ),
            ));
        }
    }

    // Exception reachability. `*` means "any" and is not checkable; a named endpoint is.
    for e in &sdc.exceptions {
        let kind = match e.kind {
            crate::sdc::ExcKind::FalsePath => "set_false_path",
            crate::sdc::ExcKind::Multicycle(_) => "set_multicycle_path",
        };
        // Every named endpoint, not the first — an exception can cut a whole bus, and each
        // member is separately capable of being a typo or of naming a path that is not there.
        let known_object = |o: &String| {
            g.knows(o)
                || clock_srcs.contains_key(o.as_str())
                || sdc.clocks.iter().any(|c| &c.name == o)
        };
        for (side, obj) in e.named_endpoints() {
            if !known_object(obj) {
                f.push(Finding::warn(
                    "exception-unknown-object",
                    format!(
                        "{kind} {side} `{obj}` names nothing in the design — \
                         that endpoint of the exception does nothing"
                    ),
                ));
            }
        }
        // Reachability. Pair by pair when both sides are named, because the answer a reader
        // needs is the specific dead pair rather than that some pair is dead.
        //
        // One-sided exceptions are checked too, and they are the common case in real SDCs —
        // `set_false_path -from [list ...]` with no `-to` at all. An earlier version skipped
        // them, which meant this check quietly did nothing on exactly the shape most designs
        // write. For those, dead means the named object sits on no timing path at all, so
        // cutting it cuts nothing.
        let named_from: Vec<&String> = e
            .from
            .iter()
            .filter(|o| o.as_str() != "*" && known_object(o))
            .collect();
        let named_to: Vec<&String> =
            e.to.iter()
                .filter(|o| o.as_str() != "*" && known_object(o))
                .collect();
        let mut dead = Vec::new();
        match (named_from.is_empty(), named_to.is_empty()) {
            (false, false) => {
                for from in &named_from {
                    for to in &named_to {
                        if !g.reaches(from, to) {
                            dead.push(format!("`{from}` -> `{to}`: no structural path"));
                        }
                    }
                }
            }
            (false, true) => {
                for from in &named_from {
                    if !g.launches_anything(from) {
                        dead.push(format!("`{from}` launches no timing path"));
                    }
                }
            }
            (true, false) => {
                for to in &named_to {
                    if !g.captures_anything(to) {
                        dead.push(format!("`{to}` captures no timing path"));
                    }
                }
            }
            (true, true) => {}
        }
        for what in dead.iter().take(PORT_LIST_CAP) {
            f.push(Finding::warn(
                "exception-unreachable",
                format!(
                    "{kind} {what} — that part of the exception is dead, and any path the \
                     author meant is still timed"
                ),
            ));
        }
        if dead.len() > PORT_LIST_CAP {
            f.push(Finding::warn(
                "exception-unreachable",
                format!(
                    "... and {} more dead {kind} endpoints ({} in total)",
                    dead.len() - PORT_LIST_CAP,
                    dead.len()
                ),
            ));
        }
    }

    // `-through` points, and `set_max_delay` / `set_min_delay`: the same two questions — does
    // each named object exist, and does it lie on a path between the sides around it.
    path_point_findings(sdc, &g, &mut f);

    // Endpoint coverage, as one number.
    let seq_total = clocked + unclocked.len();
    let out_covered = nl
        .outputs
        .iter()
        .filter(|p| out_default || out_ports.iter().any(|q| glob_match(q, p)))
        .count();
    let total = seq_total + nl.outputs.len();
    let covered = clocked + out_covered;
    let pct = if total == 0 {
        100.0
    } else {
        100.0 * covered as f64 / total as f64
    };
    f.push(Finding {
        severity: if covered < total {
            Severity::Warning
        } else {
            Severity::Info
        },
        code: "endpoint-coverage",
        message: format!(
            "endpoint coverage {pct:.1}% — {covered} of {total} constrained ({clocked}/{seq_total} \
             registers clocked, {out_covered}/{} outputs with a delay)",
            nl.outputs.len()
        ),
    });

    not_applied_findings(sdc, &mut f);
    kept_not_applied_findings(sdc, &mut f);
    crate::sdc_overlap::overlap_findings(sdc, &mut f);
    crate::sdc_clock_relations::clock_relation_findings_with_design(sdc, nl, lib, &mut f);
    sense_type_default_findings(sdc, &mut f);
    extension_findings(sdc, &mut f);
    ignored_word_findings(sdc, &mut f);
    braced_variable_findings(sdc, &mut f);
    f.sort_by(|a, b| (a.severity as u8, a.code).cmp(&(b.severity as u8, b.code)));
    LintReport { findings: f }
}

/// The checkable names in an object list: no wildcard (a pattern is not one object), no
/// `all_*` sentinel (`*INPUTS*` is every input, which cuts something whenever the design has one).
/// A clock name stands for its source.
fn checkable_names(objs: &[&crate::sdc::Objects], sdc: &Sdc) -> Vec<String> {
    let mut out = Vec::new();
    for o in objs {
        for n in &o.names {
            if n.is_empty() || n.contains(['*', '?']) {
                continue;
            }
            if o.accessor == "get_clocks" {
                if let Some(c) = sdc.clocks.iter().find(|c| &c.name == n).filter(|c| !c.source.is_empty()) {
                    out.push(c.source.clone());
                }
                continue;
            }
            out.push(n.clone());
        }
    }
    out
}

/// Which side of a path an object-valued flag names: 0 from, 1 through, 2 to.
fn side_of(flag: &str) -> Option<usize> {
    let f = flag.trim_start_matches('-').trim_start_matches("rise_").trim_start_matches("fall_");
    match f {
        "from" => Some(0),
        "through" => Some(1),
        "to" => Some(2),
        _ => None,
    }
}

/// `-through` on false and multicycle paths, and every side of `set_max_delay` / `set_min_delay`.
///
/// The exception check above judges `-from` / `-to` alone; a `-through` list narrows the
/// exception to paths through those points, and a point that is on no such path makes the whole
/// exception dead — the path the author meant stays timed (a false path) or keeps its old
/// requirement (a path delay). Per point: it must exist, and it must be reachable from the side
/// before it and reach the side after it (a register or a port when that side is open).
/// One command's path points: its label, whether it is an exception, and its sides in order
/// (`0` from, `1` through, `2` to) with their objects as written.
type PathPoints<'s> = (&'static str, bool, Vec<(usize, &'s crate::sdc::Objects)>);

fn path_point_findings(sdc: &Sdc, g: &Graph, f: &mut Vec<Finding>) {
    // (command label, is an exception, [from, through..., to] as written). An exception's
    // `-from` / `-to` objects are already checked for existence above; only its `-through` here.
    let mut cmds: Vec<PathPoints> = Vec::new();
    for e in &sdc.exceptions {
        let kind = match e.kind {
            crate::sdc::ExcKind::FalsePath => "set_false_path",
            crate::sdc::ExcKind::Multicycle(_) => "set_multicycle_path",
        };
        if e.args.iter().any(|(k, _)| side_of(k) == Some(1)) {
            cmds.push((kind, true, e.args.iter().filter_map(|(k, o)| side_of(k).map(|s| (s, o))).collect()));
        }
    }
    for p in &sdc.path_delays {
        let kind = if p.max { "set_max_delay" } else { "set_min_delay" };
        cmds.push((kind, false, p.objects.iter().filter_map(|(k, o)| side_of(k).map(|s| (s, o))).collect()));
    }
    let mut dead = Vec::new();
    for (kind, exception, sides) in &cmds {
        let of = |want: usize| -> Vec<&crate::sdc::Objects> { sides.iter().filter(|(s, _)| *s == want).map(|(_, o)| *o).collect() };
        let from = checkable_names(&of(0), sdc);
        let to = checkable_names(&of(2), sdc);
        // each `-through` is its own stage, in order
        let throughs: Vec<Vec<String>> = sides
            .iter()
            .filter(|(s, _)| *s == 1)
            .map(|(_, o)| checkable_names(&[*o], sdc))
            .collect();
        let mut stages: Vec<(&str, Vec<String>)> = vec![("-from", from)];
        stages.extend(throughs.into_iter().map(|t| ("-through", t)));
        stages.push(("-to", to));
        let mut unknown = false;
        for (flag, names) in &stages {
            if *exception && *flag != "-through" {
                unknown |= names.iter().any(|n| !g.knows(n));
                continue;
            }
            for n in names.iter().filter(|n| !g.knows(n)) {
                unknown = true;
                f.push(Finding::warn(
                    "exception-unknown-object",
                    format!("{kind} {flag} `{n}` names nothing in the design — that part of the command does nothing"),
                ));
            }
        }
        if unknown {
            continue;
        }
        // every adjacent pair of named stages must connect; an open first or last stage needs the
        // named point to reach a register or a port in that direction
        let named: Vec<&(&str, Vec<String>)> = stages.iter().filter(|(_, n)| !n.is_empty()).collect();
        for w in named.windows(2) {
            let (a, b) = (&w[0].1, &w[1].1);
            if !a.iter().any(|x| b.iter().any(|y| g.reaches(x, y))) {
                dead.push(format!("{kind}: no structural path from {} `{}` to {} `{}`", w[0].0, a.join(" "), w[1].0, b.join(" ")));
            }
        }
        if let (Some(first), true) = (named.first(), stages[0].1.is_empty()) {
            if !first.1.iter().any(|x| g.captures_anything(x)) {
                dead.push(format!("{kind}: {} `{}` is reached by no timing path", first.0, first.1.join(" ")));
            }
        }
        if let (Some(last), true) = (named.last(), stages[stages.len() - 1].1.is_empty()) {
            if !last.1.iter().any(|x| g.launches_anything(x)) {
                dead.push(format!("{kind}: {} `{}` reaches no timing endpoint", last.0, last.1.join(" ")));
            }
        }
    }
    for what in dead.iter().take(PORT_LIST_CAP) {
        f.push(Finding::warn(
            "exception-unreachable",
            format!("{what} — the command applies to no path, and any path the author meant keeps its old timing"),
        ));
    }
    if dead.len() > PORT_LIST_CAP {
        f.push(Finding::warn(
            "exception-unreachable",
            format!("... and {} more dead path commands ({} in total)", dead.len() - PORT_LIST_CAP, dead.len()),
        ));
    }
}

/// One finding per kind of command the model does not carry ([`Sdc::not_applied`]): an ERROR when
/// leaving it out can move a slack, INFO when it cannot (a synthesis-only directive).
///
/// Without these, every check above runs on what was read and says nothing about what was not, so
/// a file whose `set_disable_timing` or design-dependent queries were all skipped could come back
/// clean. A clean report must mean the whole file was looked at.
const SENSE_DATA_TAIL: &str = "not modelled, not applied; the reference timer does not apply it either";

fn not_applied_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    for n in sdc.not_applied() {
        let times = if n.count == 1 { String::new() } else { format!(" ({} commands)", n.count) };
        if n.command == "set_sense" && n.why.ends_with(crate::sdc::SENSE_TYPE_DATA) {
            // not applied here or by the reference: the two agree, so a warning
            f.push(Finding::warn("sdc-not-applied", format!("`set_sense -type data`{times}: {SENSE_DATA_TAIL}")));
        } else if n.affects_timing {
            f.push(Finding::err(
                "sdc-not-applied",
                format!(
                    "`{}`{times}: {} — it can change timing and nothing here checked it",
                    n.command, n.why
                ),
            ));
        } else {
            f.push(Finding {
                severity: Severity::Info,
                code: "sdc-not-applied",
                message: format!("`{}`{times}: {} (no effect on timing)", n.command, n.why),
            });
        }
    }
}

/// One finding per kind of command kept and written back that no checker or timer here applies
/// ([`Sdc::kept_not_applied`]): a WARNING when applying it could move a slack, so the report is not
/// called clean while it goes unapplied, and INFO when it could not (a power or area directive).
/// Never an error: the command was read as the reference reads it, and `write` writes it back.
/// Each reference-timer extension the file uses ([`Sdc::extensions`]), with its count: read and
/// applied like the rest, but not SDC, so another SDC reader may refuse the file. Info only.
fn extension_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    let mut seen: Vec<(&str, usize)> = Vec::new();
    for e in &sdc.extensions {
        match seen.iter_mut().find(|(n, _)| n == e) {
            Some((_, c)) => *c += 1,
            None => seen.push((e, 1)),
        }
    }
    for (e, n) in seen {
        let times = if n == 1 { String::new() } else { format!(" ({n} uses)") };
        f.push(Finding { severity: Severity::Info, code: "sdc-extension", message: format!("`{e}`{times}: a reference-timer extension no SDC version declares") });
    }
}

/// Each word the reference warns about and ignores while applying its command
/// ([`Sdc::ignored_words`]), with its count. A WARNING: the file says something the reference
/// does not do — `-include_generated_clocks` puts no generated clock in any group, so those
/// clocks stay timed against the other groups.
fn ignored_word_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    let mut seen: Vec<(&str, usize)> = Vec::new();
    for e in &sdc.ignored_words {
        match seen.iter_mut().find(|(n, _)| n == e) {
            Some((_, c)) => *c += 1,
            None => seen.push((e, 1)),
        }
    }
    for (e, n) in seen {
        let times = if n == 1 { String::new() } else { format!(" ({n} uses)") };
        let what = match e.rsplit(' ').next() {
            Some("-include_generated_clocks") => {
                "ignored by the reference timer, which applies the groups without it: each group holds only the clocks \
                 it names, and their generated clocks are timed against the other groups"
            }
            _ if e.starts_with("set_clock_groups -group ") => {
                "names no clock defined before it; the reference timer leaves it out of the group (a group left \
                 empty is not made, and one group left alone separates its clocks from every other clock)"
            }
            Some(w) if w.starts_with('-') => "not an option; the reference timer warns, ignores it and applies the rest",
            _ => "an extra word; the reference timer warns, ignores it and applies the rest",
        };
        f.push(Finding::warn("sdc-word-ignored", format!("`{e}`{times}: {what}")));
    }
}

/// The `set_sense` commands written without `-type` ([`Sdc::senses_type_defaulted`]), with their
/// count: read as clock, the SDC 2.1 default, where the reference timer applies none of them.
/// Info only — the file is SDC as written; the note is that the two readings differ.
fn sense_type_default_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    let n = sdc.senses_type_defaulted.len();
    if n == 0 {
        return;
    }
    let times = if n == 1 { String::new() } else { format!(" ({n} commands)") };
    f.push(Finding {
        severity: Severity::Info,
        code: "sdc-sense-type-default",
        message: format!("`set_sense`{times} no -type: applied as clock (SDC 2.1 default); the reference timer ignores it without -type"),
    });
}

fn kept_not_applied_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    for k in sdc.kept_not_applied() {
        let times = if k.count == 1 { String::new() } else { format!(" ({} commands)", k.count) };
        let (severity, tail) = if k.affects_timing {
            (Severity::Warning, "it can change timing, and no checker or timer here applies it")
        } else {
            (Severity::Info, "no checker or timer here applies it (no effect on timing)")
        };
        f.push(Finding { severity, code: "sdc-kept-not-applied", message: format!("`{}`{times}: kept and written back; {tail}", k.command) });
    }
}

/// The checks that need no design: what the reader could not apply ([`Sdc::not_applied`]) and
/// the clock definitions themselves (a non-positive period, one name defined twice). Every
/// structural check — I/O coverage, clock reach, exception reachability, endpoint coverage —
/// needs the netlist, and an INFO finding says they did not run, so the report cannot be read as
/// a full check.
/// An object name that is a variable reference left whole (`${MAIN_CLK_PIN}`): Tcl substitutes
/// nothing inside braces, so `remove_from_collection [all_inputs] {${MAIN_CLK_PIN}}` names a port
/// literally called `${MAIN_CLK_PIN}` and removes nothing — the clock port keeps its input delay.
/// A warning: the file reads as Tcl reads it, which is almost never what its author meant.
fn braced_variable_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    let looks_like_var = |n: &str| {
        let n = n.strip_prefix('$').unwrap_or("");
        let n = n.strip_prefix('{').and_then(|x| x.strip_suffix('}')).unwrap_or(n);
        !n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == ':')
    };
    let mut seen: BTreeMap<String, &str> = BTreeMap::new();
    let delays = sdc.input_delays.iter().map(|d| ("set_input_delay", &d.ports)).chain(sdc.output_delays.iter().map(|d| ("set_output_delay", &d.ports)));
    let exceptions = sdc.exceptions.iter().flat_map(|e| [("exception", &e.from), ("exception", &e.to)]);
    for (cmd, names) in delays.chain(exceptions) {
        for n in names.iter().filter(|n| looks_like_var(n)) {
            seen.entry(n.clone()).or_insert(cmd);
        }
    }
    for (n, cmd) in seen {
        f.push(Finding::warn(
            "braced-variable",
            format!(
                "{cmd} names the object `{n}`: Tcl does not substitute a variable inside braces, so this is \
                 the literal name, not the variable's value"
            ),
        ));
    }
}

pub fn lint_without_design(sdc: &Sdc) -> LintReport {
    let mut f = Vec::new();
    let mut by_name: BTreeMap<&str, u32> = BTreeMap::new();
    for c in sdc.clocks.iter().chain(&sdc.superseded_clocks) {
        *by_name.entry(c.name.as_str()).or_default() += 1;
        if c.period <= 0.0 {
            f.push(Finding::err(
                "clock-period",
                format!("clock `{}` has a non-positive period ({} ns)", c.name, c.period),
            ));
        }
    }
    for (name, n) in by_name {
        if n > 1 {
            f.push(Finding::err("dup-clock-name", format!("clock `{name}` is defined {n} times")));
        }
    }
    not_applied_findings(sdc, &mut f);
    kept_not_applied_findings(sdc, &mut f);
    crate::sdc_overlap::overlap_findings(sdc, &mut f);
    crate::sdc_clock_relations::clock_relation_findings(sdc, &mut f);
    sense_type_default_findings(sdc, &mut f);
    extension_findings(sdc, &mut f);
    ignored_word_findings(sdc, &mut f);
    braced_variable_findings(sdc, &mut f);
    f.push(Finding {
        severity: Severity::Info,
        code: "design-checks-not-run",
        message: "no netlist: I/O coverage, clock reach, exception reachability and endpoint \
                  coverage were not checked"
            .into(),
    });
    f.sort_by(|a, b| (a.severity as u8, a.code).cmp(&(b.severity as u8, b.code)));
    LintReport { findings: f }
}

/// A local forward/backward view of the netlist, built once and shared by the depth passes.
///
/// Deliberately local rather than borrowed from the timing engine: this module's contract is
/// that it runs *independent of the timing run*, so a constraint problem stays reportable
/// without first computing slack from constraints we already suspect.
struct Graph {
    driver: BTreeMap<String, Option<usize>>, // net -> driving instance (None = primary input)
    sinks: BTreeMap<String, Vec<usize>>,     // net -> instances with it on an input pin
    is_seq: Vec<bool>,
    known: BTreeSet<String>, // every net, port and instance name that exists
    inputs: BTreeSet<String>,
    outputs: BTreeSet<String>,
    inst_index: BTreeMap<String, usize>,
    conns: Vec<Vec<(String, String)>>,
    out_pins: Vec<Vec<String>>,
    /// Per instance, the pins it has: the ones it connects and, for a library cell, every pin
    /// the cell defines (an unconnected `QN` is still a pin).
    pins: Vec<BTreeSet<String>>,
}

impl Graph {
    fn build(nl: &Netlist, lib: &Lib) -> Graph {
        let mut g = Graph {
            driver: BTreeMap::new(),
            sinks: BTreeMap::new(),
            is_seq: Vec::new(),
            known: BTreeSet::new(),
            inputs: nl.inputs.iter().cloned().collect(),
            outputs: nl.outputs.iter().cloned().collect(),
            inst_index: BTreeMap::new(),
            conns: Vec::new(),
            out_pins: Vec::new(),
            pins: Vec::new(),
        };
        for p in nl.inputs.iter().chain(nl.outputs.iter()) {
            g.driver.entry(p.clone()).or_insert(None);
            g.known.insert(p.clone());
        }
        for (i, inst) in nl.insts.iter().enumerate() {
            let cell = lib.cells.get(&inst.cell);
            g.is_seq.push(cell.map(|c| c.is_seq).unwrap_or(false));
            g.inst_index.insert(inst.name.clone(), i);
            g.known.insert(inst.name.clone());
            g.conns.push(inst.conns.clone());
            let mut pins: BTreeSet<String> = inst.conns.iter().map(|(p, _)| p.clone()).collect();
            if let Some(c) = cell {
                pins.extend(c.pins.keys().cloned());
            }
            g.pins.push(pins);
            let mut outs = Vec::new();
            for (pin, net) in &inst.conns {
                g.known.insert(net.clone());
                if cell.and_then(|c| c.pins.get(pin)).map(|p| p.direction) == Some(Dir::Out) {
                    g.driver.insert(net.clone(), Some(i));
                    outs.push(pin.clone());
                } else {
                    g.sinks.entry(net.clone()).or_default().push(i);
                }
            }
            g.out_pins.push(outs);
        }
        g
    }

    /// `inst/pin` paths resolve on the instance half.
    ///
    /// ⛔ The pin half must be a pin of that instance. Accepted on the instance alone, a pin inside
    /// a block kept whole (`u_b/r/D`, read at instance level as `u_b/r`) passed on `u_b` — an
    /// exception naming nothing, reported as fine.
    fn knows(&self, obj: &str) -> bool {
        if self.known.contains(obj) {
            return true;
        }
        let inst = instance_of(obj);
        let Some(&i) = self.inst_index.get(inst).filter(|_| inst != obj) else { return false };
        obj.get(inst.len() + 1..).is_some_and(|pin| self.pins[i].contains(pin))
    }

    /// The instance an SDC object names: the object ITSELF when it is an instance (an exception
    /// side is kept at instance level, `u_b/r`), else the instance part of a pin (`u_b/r/D`).
    /// ⛔ Stripped unconditionally, a hierarchical instance `u_b/r` became `u_b`, which is no
    /// instance, and every exception on a flattened block's register read as dead.
    fn base_of<'o>(&self, obj: &'o str) -> &'o str {
        if self.inst_index.contains_key(obj) {
            obj
        } else {
            instance_of(obj)
        }
    }

    fn net_at(&self, inst: usize, pin: &str) -> Option<&str> {
        self.conns[inst]
            .iter()
            .find(|(p, _)| p == pin)
            .map(|(_, n)| n.as_str())
    }

    /// Walk a clock net back through combinational cells to a declared SDC clock source.
    fn trace_to_source<'a>(&self, net: &str, srcs: &BTreeMap<&str, &'a str>) -> Option<&'a str> {
        let mut seen = BTreeSet::new();
        let mut stack = vec![net.to_string()];
        while let Some(n) = stack.pop() {
            if let Some(d) = srcs.get(n.as_str()) {
                return Some(d);
            }
            if !seen.insert(n.clone()) {
                continue;
            }
            let Some(Some(i)) = self.driver.get(&n).copied() else {
                continue; // a primary input that is not a clock source, or undriven
            };
            if self.is_seq[i] {
                continue; // divided/gated off a flop — not traced in v0
            }
            for (pin, nn) in &self.conns[i] {
                if !self.out_pins[i].contains(pin) {
                    stack.push(nn.clone());
                }
            }
        }
        None
    }

    /// The nets an SDC object launches from: a port is itself, an instance is its outputs.
    fn start_nets(&self, obj: &str) -> Vec<String> {
        let base = self.base_of(obj);
        if let Some(&i) = self.inst_index.get(base) {
            return self.out_pins[i]
                .iter()
                .filter_map(|p| self.net_at(i, p).map(str::to_string))
                .collect();
        }
        if self.known.contains(obj) {
            return vec![obj.to_string()];
        }
        Vec::new()
    }

    /// Does `from` launch any timing path at all — reaching a register or an output port?
    ///
    /// A `-from` naming an object that drives nothing cuts nothing. This is what makes a
    /// one-sided exception checkable, and one-sided is how real SDCs are usually written.
    fn launches_anything(&self, from: &str) -> bool {
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut stack = self.start_nets(from);
        while let Some(n) = stack.pop() {
            if !seen.insert(n.clone()) {
                continue;
            }
            if self.outputs.contains(&n) {
                return true;
            }
            for &i in self.sinks.get(&n).into_iter().flatten() {
                if self.is_seq[i] {
                    return true; // reached a register: a real endpoint
                }
                for (pin, nn) in &self.conns[i] {
                    if self.out_pins[i].contains(pin) {
                        stack.push(nn.clone());
                    }
                }
            }
        }
        false
    }

    /// Is `to` reached by any timing path — walking back to a register or a primary input?
    fn captures_anything(&self, to: &str) -> bool {
        let base = self.base_of(to);
        // Start from the object's input nets (an instance) or the net itself (a port).
        let mut stack: Vec<String> = match self.inst_index.get(base) {
            Some(&i) => self.conns[i]
                .iter()
                .filter(|(p, _)| !self.out_pins[i].contains(p))
                .map(|(_, n)| n.clone())
                .collect(),
            None => vec![to.to_string()],
        };
        let mut seen: BTreeSet<String> = BTreeSet::new();
        while let Some(n) = stack.pop() {
            if !seen.insert(n.clone()) {
                continue;
            }
            match self.driver.get(&n) {
                // An undriven net is only a launch point if it is a primary INPUT. An output
                // port with no driver is dangling — nothing reaches it, which is exactly the
                // case worth reporting rather than passing.
                Some(None) => {
                    if self.inputs.contains(&n) {
                        return true;
                    }
                }
                Some(Some(i)) => {
                    if self.is_seq[*i] {
                        return true; // a register launches
                    }
                    for (pin, nn) in &self.conns[*i] {
                        if !self.out_pins[*i].contains(pin) {
                            stack.push(nn.clone());
                        }
                    }
                }
                None => {}
            }
        }
        false
    }

    /// Is there a structural path from `from` to `to`? Forward through combinational logic,
    /// stopping at sequential cells — which is what a timing path is.
    fn reaches(&self, from: &str, to: &str) -> bool {
        let target_base = self.base_of(to);
        let target_inst = self.inst_index.get(target_base).copied();
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut stack = self.start_nets(from);
        while let Some(n) = stack.pop() {
            if n == to || n == target_base {
                return true;
            }
            if !seen.insert(n.clone()) {
                continue;
            }
            for &i in self.sinks.get(&n).into_iter().flatten() {
                if Some(i) == target_inst {
                    return true;
                }
                if self.is_seq[i] {
                    continue; // a register ends the path
                }
                for (pin, nn) in &self.conns[i] {
                    if self.out_pins[i].contains(pin) {
                        stack.push(nn.clone());
                    }
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("tests/data/sdc_lint/seq.lib").expect("seq.lib")
    }
    fn nl() -> Netlist {
        // a registered path: in -> DFF -> out, plus the clock port
        crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\nwire q;\n\
             DFF r(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }

    #[test]
    fn clean_sdc_lints_clean() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        assert_eq!(r.warnings(), 0, "{:?}", r.findings);
    }

    /// `set_current_design` (SDC 1.1 in the vendor releases of the parser) is read as
    /// `current_design`: the file is read on, and `check` warns only when the design it names is
    /// not the netlist's top. The constraints are applied to the top either way.
    #[test]
    fn set_current_design_is_current_design_and_warns_on_another_design() {
        let base = "create_clock -name clk -period 10 [get_ports clk]\nset_input_delay 1 -clock clk [all_inputs]\nset_output_delay 1 -clock clk [all_outputs]\n";
        for cmd in ["set_current_design", "current_design"] {
            let sdc = Sdc::parse(&format!("{cmd} t\n{base}")).unwrap_or_else(|e| panic!("{cmd}: {e:?}"));
            assert_eq!(sdc.current_design.as_deref(), Some("t"), "{cmd}");
            assert_eq!(sdc.clocks.len(), 1, "{cmd}: the read stopped");
            assert!(sdc.rejected.is_empty() && sdc.not_applied().is_empty() && sdc.extensions.is_empty(), "{cmd}: {:?}", sdc.not_applied());
            let r = lint(&nl(), &sdc, &lib());
            assert_eq!(r.warnings(), 0, "{cmd}: {:?}", r.findings);
            let other = Sdc::parse(&format!("{cmd} other\n{base}")).unwrap();
            let r = lint(&nl(), &other, &lib());
            let w: Vec<_> = r.findings.iter().filter(|f| f.code == "current-design-not-top").collect();
            assert_eq!(w.len(), 1, "{cmd}: {:?}", r.findings);
            assert_eq!(w[0].severity, Severity::Warning);
            assert_eq!(w[0].message, "`current_design other` names a design other than the netlist's top `t`; the constraints are applied to `t`");
            assert_eq!(r.errors(), 0, "{cmd}: {:?}", r.findings);
        }
        // a quoted name is the name: Tcl strips the braces or quotes before the command sees it,
        // so `{t}` is the top and gives no warning
        for cmd in ["set_current_design {t}", "current_design {t}", "set_current_design \"t\"", "current_design \"t\""] {
            let sdc = Sdc::parse(&format!("{cmd}\n{base}")).unwrap_or_else(|e| panic!("{cmd}: {e:?}"));
            assert_eq!(sdc.current_design.as_deref(), Some("t"), "{cmd}");
            let r = lint(&nl(), &sdc, &lib());
            assert_eq!(r.warnings(), 0, "{cmd}: {:?}", r.findings);
        }
        // written back in the reference timer's spelling
        let w = Sdc::parse(&format!("set_current_design t\n{base}")).unwrap().write(&crate::sdc_write::WriteOpts::default());
        assert!(w.text.contains("current_design t") && !w.text.contains("set_current_design"), "{}", w.text);
    }

    #[test]
    fn registers_without_a_clock_is_an_error() {
        let sdc = Sdc::parse("set_input_delay 1 [all_inputs]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r
            .findings
            .iter()
            .any(|f| f.code == "no-clock" && f.severity == Severity::Error));
    }

    /// A zero period never reaches the lint: the reader refuses it, as the reference does
    /// (`check_positive_float`, Error 572).
    #[test]
    fn a_zero_period_is_refused_at_read() {
        let e = Sdc::parse("create_clock -name clk -period 0 [get_ports clk]\n").unwrap_err();
        assert!(e.to_string().contains("positive float"), "{e}");
    }

    #[test]
    fn dup_clock_name_is_an_error() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 5 [get_ports clk]\n\
             create_clock -name clk -period 10 [get_ports clk]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "dup-clock-name"));
    }

    #[test]
    fn unconstrained_io_and_bad_port_warn() {
        // clock present, but no input/output delays at all, and a stray clock source
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "unconstrained-input")); // din
        assert!(r.findings.iter().any(|f| f.code == "unconstrained-output")); // dout
        assert_eq!(r.errors(), 0);
    }

    #[test]
    fn clock_on_missing_port_warns() {
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports nope]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "clock-source"));
    }

    const CLEAN: &str = "create_clock -name clk -period 10 [get_ports clk]\n\
                         set_input_delay 1 -clock clk [all_inputs]\n\
                         set_output_delay 1 -clock clk [all_outputs]\n";

    /// A command the model does not carry and that can move a slack makes the check FAIL: a clean
    /// report has to mean every constraint was looked at.
    #[test]
    fn an_unmodelled_timing_command_is_an_error_not_silence() {
        let sdc = Sdc::parse(&format!("{CLEAN}set_assigned_delay -cell -from u1/A -to u1/Z 0.1\n")).unwrap();
        let r = lint(&nl(), &sdc, &lib());
        let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-not-applied").collect();
        assert_eq!(f.len(), 1, "{:?}", r.findings);
        assert_eq!(f[0].severity, Severity::Error);
        assert!(f[0].message.contains("set_assigned_delay"), "{}", f[0].message);
    }

    /// A `set_sense` without `-type` is read as clock, the SDC 2.1 default, and check says so at
    /// info level with its count — the reference timer applies nothing for it. One with
    /// `-type clock`, and `set_clock_sense`, are not counted.
    #[test]
    fn a_sense_without_a_type_is_reported_as_read_as_clock() {
        let text = "set_sense -stop_propagation [get_pins u1/Y]\nset_sense -positive -clocks [get_clocks c] [get_pins u2/Y]\n\
                    set_sense -type clock -negative [get_pins u3/Y]\nset_clock_sense -positive [get_pins u4/Y]\n";
        let r = lint_without_design(&Sdc::parse(text).unwrap());
        let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-sense-type-default").collect();
        assert_eq!(f.len(), 1, "{:?}", r.findings);
        assert_eq!(f[0].severity, Severity::Info);
        assert_eq!(
            f[0].message,
            "`set_sense` (2 commands) no -type: applied as clock (SDC 2.1 default); the reference timer ignores it without -type"
        );
        let one = lint_without_design(&Sdc::parse("set_sense -stop_propagation [get_pins u1/Y]\n").unwrap());
        let f: Vec<_> = one.findings.iter().filter(|f| f.code == "sdc-sense-type-default").map(|f| f.message.as_str()).collect();
        assert_eq!(f, ["`set_sense` no -type: applied as clock (SDC 2.1 default); the reference timer ignores it without -type"]);
        let typed = lint_without_design(&Sdc::parse("set_sense -type clock -negative [get_pins u3/Y]\n").unwrap());
        assert!(!typed.findings.iter().any(|f| f.code == "sdc-sense-type-default"), "{:?}", typed.findings);
    }

    /// `set_sense -type data` is not modelled: a WARNING, not an error, since the reference timer
    /// does not apply it either (its Warning 413). The command is not written back.
    #[test]
    fn a_data_sense_is_a_warning() {
        let s = Sdc::parse("set_sense -type data -positive [get_pins u3/Y]\nset_sense -type data -negative [get_pins u4/Y]\n").unwrap();
        let r = lint_without_design(&s);
        let f: Vec<(Severity, &str)> = r.findings.iter().filter(|f| f.code == "sdc-not-applied").map(|f| (f.severity, f.message.as_str())).collect();
        assert_eq!(
            f,
            [(Severity::Warning, "`set_sense -type data` (2 commands): not modelled, not applied; the reference timer does not apply it either")]
        );
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        assert!(s.senses.is_empty());
        // refused, so not also counted as kept
        assert!(s.kept_not_applied().is_empty(), "{:?}", s.kept_not_applied());
    }

    /// A command kept and written back that no checker or timer here applies is REPORTED, with its
    /// count: a warning when applying it could move a slack (SDC's `set_clock_gating_check` adds a
    /// setup/hold check on a gating enable), info when it could not (`set_max_area` steers
    /// synthesis). Never an error, and never silence: before this, a file of kept-only commands
    /// came back CLEAN. A command a timer here applies (`set_data_check`, by sta-si) is not one.
    #[test]
    fn a_kept_command_nothing_applies_is_reported_with_its_count() {
        let text = format!(
            "{CLEAN}set_clock_gating_check -setup 0.1 [get_clocks clk]\nset_clock_gating_check -hold 0.1 [get_clocks clk]\n\
             set_max_area 0\nset_data_check -from [get_pins u1/A] -to [get_pins u2/A] 0.1\n"
        );
        let sdc = Sdc::parse(&text).unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-kept-not-applied").collect();
        assert_eq!(f.len(), 2, "{:?}", r.findings);
        assert_eq!(f[0].severity, Severity::Warning);
        assert!(f[0].message.contains("`set_clock_gating_check` (2 commands)"), "{}", f[0].message);
        assert_eq!(f[1].severity, Severity::Info);
        assert!(f[1].message.contains("`set_max_area`"), "{}", f[1].message);
        // and without a design too
        let r = lint_without_design(&sdc);
        assert_eq!(r.findings.iter().filter(|f| f.code == "sdc-kept-not-applied").count(), 2, "{:?}", r.findings);
    }

    /// The kept timing commands outside the synthesis directives are each a WARNING: a latch's
    /// borrow limit (`set_max_time_borrow`), a path margin (`set_path_margin`), a PVT
    /// (`set_pvt`). A file whose only extra constraints are these is not reported clean.
    #[test]
    fn kept_latch_margin_and_pvt_commands_are_warnings() {
        let text = format!(
            "{CLEAN}set_max_time_borrow 0.5 [get_clocks clk]\nset_path_margin 0.1 -to [get_ports dout]\nset_pvt -voltage 1.0 [get_cells r]\n"
        );
        let sdc = Sdc::parse(&text).unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        for c in ["set_max_time_borrow", "set_path_margin", "set_pvt"] {
            let found = r.findings.iter().any(|f| f.code == "sdc-kept-not-applied" && f.severity == Severity::Warning && f.message.contains(&format!("`{c}`")));
            assert!(found, "{c}: {:?}", r.findings);
        }
    }

    /// A query that needs the design and was not evaluated is a rejected command: an error too.
    #[test]
    fn an_unevaluated_query_is_an_error() {
        let sdc = Sdc::parse(&format!("{CLEAN}set_false_path -to [get_pins -filter {{x==1}} *]\n")).unwrap();
        assert!(!sdc.unevaluated_queries.is_empty(), "{:?}", sdc.rejected);
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "sdc-not-applied" && f.severity == Severity::Error), "{:?}", r.findings);
    }

    /// A synthesis-only directive is reported, but as INFO: it cannot move a slack, so it must
    /// not fail the check.
    #[test]
    fn a_benign_command_is_info_and_does_not_fail() {
        let sdc = Sdc::parse(&format!("{CLEAN}set_dont_touch [get_cells u1]\nset_dont_touch [get_cells u2]\n")).unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-not-applied").collect();
        assert_eq!(f.len(), 1, "grouped, not one per command: {:?}", r.findings);
        assert_eq!(f[0].severity, Severity::Info);
        assert!(f[0].message.contains("(2 commands)"), "{}", f[0].message);
    }

    /// Kinds that are modelled in their own records are not reported as not applied.
    #[test]
    fn modelled_kinds_are_not_reported() {
        let sdc = Sdc::parse(&format!("{CLEAN}set_max_delay 2 -from [get_ports din]\nset_case_analysis 0 din\n")).unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(!r.findings.iter().any(|f| f.code == "sdc-not-applied"), "{:?}", r.findings);
    }

    /// Without a design the report says the structural checks did not run, and still reports
    /// what was not applied.
    #[test]
    fn without_a_design_the_report_says_what_did_not_run() {
        let sdc = Sdc::parse(&format!("{CLEAN}set_assigned_delay -cell -from u1/A -to u1/Z 0.1\n")).unwrap();
        let r = lint_without_design(&sdc);
        assert!(r.findings.iter().any(|f| f.code == "design-checks-not-run" && f.severity == Severity::Info));
        assert_eq!(r.errors(), 1, "{:?}", r.findings);
    }
}

#[cfg(test)]
mod depth_tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("tests/data/sdc_lint/seq.lib").expect("seq.lib")
    }

    /// Two registers in series plus a combinational gate, so there is a real path to reason
    /// about: `din -> ra -> (gate) -> rb -> dout`.
    fn chain() -> Netlist {
        crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(mid));\n\
             DFF rb(.CK(clk),.D(mid),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }

    fn base_sdc() -> String {
        "create_clock -name clk -period 10 [get_ports clk]\n\
         set_input_delay 1 -clock clk [all_inputs]\n\
         set_output_delay 1 -clock clk [all_outputs]\n"
            .into()
    }

    fn has(r: &LintReport, code: &str) -> bool {
        r.findings.iter().any(|f| f.code == code)
    }

    #[test]
    fn an_exception_naming_nothing_is_reported() {
        // The failure this exists for: a typo'd exception does not error, it silently does
        // nothing, and the path the author meant stays timed.
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from ra -to no_such_register\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            has(&r, "exception-unknown-object"),
            "typo'd -to must be reported: {:?}",
            r.findings
        );
    }

    #[test]
    fn an_exception_on_a_real_path_is_quiet() {
        let sdc = Sdc::parse(&format!("{}set_false_path -from ra -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            !has(&r, "exception-unreachable") && !has(&r, "exception-unknown-object"),
            "ra -> rb is a real path and must not be flagged: {:?}",
            r.findings
        );
    }

    #[test]
    fn an_exception_between_two_real_objects_with_no_path_is_reported() {
        // Both ends exist, so the existence check passes — only reachability catches this.
        // Backwards: there is no path from rb to ra.
        let sdc = Sdc::parse(&format!("{}set_false_path -from rb -to ra\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "rb -> ra does not exist and the exception is dead: {:?}",
            r.findings
        );
    }

    /// `-through` was invisible to the reachability check: an exception was judged on `-from` /
    /// `-to` alone, so a `-through` on no path between them passed.
    #[test]
    fn a_through_point_on_the_path_is_quiet() {
        let sdc = Sdc::parse(&format!("{}set_false_path -from ra -through mid -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(!has(&r, "exception-unreachable") && !has(&r, "exception-unknown-object"), "{:?}", r.findings);
    }

    #[test]
    fn a_through_point_off_the_path_is_reported() {
        // rb -> dout exists, but `mid` is before rb, not between rb and dout
        let sdc = Sdc::parse(&format!("{}set_false_path -from rb -through mid -to dout\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(has(&r, "exception-unreachable"), "{:?}", r.findings);
    }

    #[test]
    fn a_through_point_naming_nothing_is_reported_once() {
        let sdc = Sdc::parse(&format!("{}set_false_path -from ra -through nope -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        let n = r.findings.iter().filter(|f| f.code == "exception-unknown-object").count();
        assert_eq!(n, 1, "{:?}", r.findings);
    }

    /// `set_max_delay` / `set_min_delay` were never linted at all.
    #[test]
    fn a_path_delay_on_a_real_path_is_quiet() {
        let sdc = Sdc::parse(&format!("{}set_max_delay 2 -from ra -to rb\nset_min_delay 0 -from ra -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(!has(&r, "exception-unreachable") && !has(&r, "exception-unknown-object"), "{:?}", r.findings);
    }

    #[test]
    fn a_path_delay_with_no_path_or_a_typo_is_reported() {
        let sdc = Sdc::parse(&format!("{}set_max_delay 2 -from rb -to ra\n", base_sdc())).unwrap();
        assert!(has(&lint(&chain(), &sdc, &lib()), "exception-unreachable"));
        let sdc = Sdc::parse(&format!("{}set_max_delay 2 -to no_such_reg\n", base_sdc())).unwrap();
        assert!(has(&lint(&chain(), &sdc, &lib()), "exception-unknown-object"));
    }

    /// An open side is any register or port: a one-sided path delay on a point that captures
    /// nothing is dead.
    #[test]
    fn a_one_sided_path_delay_is_checked() {
        let n = crate::netlist::parse(
            "module t(clk,din,dout,x);\ninput clk,din; output dout,x;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!("{}set_max_delay 2 -to x\n", base_sdc())).unwrap();
        assert!(has(&lint(&n, &sdc, &lib()), "exception-unreachable"));
        let sdc = Sdc::parse(&format!("{}set_max_delay 2 -to dout\n", base_sdc())).unwrap();
        assert!(!has(&lint(&n, &sdc, &lib()), "exception-unreachable"));
    }

    #[test]
    fn a_clock_that_reaches_nothing_leaves_registers_untimed() {
        // The SDC looks populated — a clock is defined — but it is on a port that drives no
        // register, so every register is untimed. A design-wide "registers but no clocks"
        // check cannot see this, which is the point of tracing.
        let n = crate::netlist::parse(
            "module t(clk,other,din,dout);\ninput clk,other,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(
            "create_clock -name c -period 10 [get_ports other]\n\
             set_input_delay 1 -clock c [all_inputs]\n\
             set_output_delay 1 -clock c [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "register-no-clock-reaches"),
            "a clock on the wrong port leaves ra untimed: {:?}",
            r.findings
        );
        assert!(
            !has(&r, "no-clock"),
            "a clock IS defined — the old check stays quiet"
        );
    }

    #[test]
    fn coverage_is_reported_and_is_complete_on_a_fully_constrained_design() {
        let sdc = Sdc::parse(&base_sdc()).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        let cov = r
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")
            .expect("coverage is always reported, not only when it is bad");
        assert_eq!(
            cov.severity,
            Severity::Info,
            "complete coverage is not a warning"
        );
        assert!(cov.message.contains("100.0%"), "{}", cov.message);
    }

    #[test]
    fn coverage_falls_when_an_output_is_unconstrained() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n",
        )
        .unwrap();
        let r = lint(&chain(), &sdc, &lib());
        let cov = r
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")
            .unwrap();
        assert_eq!(cov.severity, Severity::Warning);
        assert!(!cov.message.contains("100.0%"), "{}", cov.message);
    }

    #[test]
    fn a_one_sided_exception_on_a_dead_port_is_reported() {
        // The shape real SDCs actually write: `-from [list ...]` with no `-to`. An earlier
        // version only checked when BOTH sides were named, so this — the common case — was
        // silently never checked at all.
        let n = crate::netlist::parse(
            "module t(clk,din,unused,dout);\ninput clk,din,unused; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from [get_ports {{unused}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "`unused` drives nothing, so cutting it cuts nothing: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_one_sided_exception_on_a_live_port_is_quiet() {
        let n = crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from [get_ports {{din}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            !has(&r, "exception-unreachable"),
            "`din` reaches a register — a real cut, not a dead one: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_one_sided_to_exception_is_checked_from_the_other_direction() {
        let n = crate::netlist::parse(
            "module t(clk,din,dangling,dout);\ninput clk,din; output dangling,dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -to [get_ports {{dangling}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "`dangling` is driven by nothing, so nothing is cut: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_wildcard_exception_is_not_checkable_and_is_not_flagged() {
        // `-from *` means "any"; reporting it as unknown would be noise on a legitimate SDC.
        let sdc = Sdc::parse(&format!("{}set_false_path -from * -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            !has(&r, "exception-unknown-object") && !has(&r, "exception-unreachable"),
            "a wildcard is not a typo: {:?}",
            r.findings
        );
    }
}

#[cfg(test)]
mod accessor_tests {
    use super::*;

    #[test]
    fn the_coverage_accessor_reads_back_what_the_finding_says() {
        // The accessor derives from the finding rather than storing a second copy, so this
        // guards the one way they could drift: a change to the message wording.
        let r = LintReport {
            findings: vec![Finding {
                severity: Severity::Warning,
                code: "endpoint-coverage",
                message: "endpoint coverage 66.7% — 2 of 3 constrained (1/2 registers \
                          clocked, 1/1 outputs with a delay)"
                    .into(),
            }],
        };
        assert_eq!(r.endpoint_coverage_pct(), Some(66.7));
    }

    #[test]
    fn no_coverage_finding_means_no_number_rather_than_a_wrong_one() {
        let r = LintReport {
            findings: vec![Finding::warn("something-else", "unrelated".into())],
        };
        assert_eq!(r.endpoint_coverage_pct(), None);
    }
}

#[cfg(test)]
mod noise_tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("tests/data/sdc_lint/seq.lib").expect("seq.lib")
    }

    /// A wide top-level wrapper: many outputs, none constrained.
    fn wide(n: usize) -> Netlist {
        let ports: Vec<String> = (0..n).map(|i| format!("o{i}")).collect();
        let decl = ports.join(",");
        crate::netlist::parse(&format!(
            "module t(clk,din,{decl});\ninput clk,din; output {decl};\n\
             DFF r(.CK(clk),.D(din),.Q(o0));\nendmodule\n"
        ))
        .unwrap()
    }

    #[test]
    fn hundreds_of_unconstrained_ports_collapse_to_a_count() {
        // Found on a real pad wrapper: 484 unconstrained outputs produced 484 warnings, and
        // the two findings that mattered were somewhere in the middle of them. A reader
        // scrolls past that, which is how a checker stops being run.
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&wide(200), &sdc, &lib());
        let outs: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.code == "unconstrained-output")
            .collect();
        assert!(
            outs.len() <= PORT_LIST_CAP + 1,
            "listed {} findings for 200 ports — the cap is not applied",
            outs.len()
        );
        let summary = outs.last().expect("a summary line");
        assert!(
            summary.message.contains("200 in total"),
            "the count must survive the cap: {}",
            summary.message
        );
    }

    #[test]
    fn a_small_design_still_names_every_port() {
        // The cap must not cost detail where detail is usable.
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&wide(3), &sdc, &lib());
        let outs: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.code == "unconstrained-output")
            .collect();
        assert_eq!(outs.len(), 3, "three ports, three named findings");
        assert!(outs.iter().all(|f| !f.message.contains("in total")));
    }
}

#[cfg(test)]
mod glob_tests {
    use super::*;

    #[test]
    fn bus_wildcards_match_their_members() {
        // `[get_ports {paddr_i[*]}]` is how a real SDC constrains a whole bus. Comparing it
        // as a literal string reported every such line as targeting a nonexistent port.
        assert!(glob_match("paddr_i[*]", "paddr_i[0]"));
        assert!(glob_match("paddr_i[*]", "paddr_i[15]"));
        assert!(!glob_match("paddr_i[*]", "pwdata_i[0]"));
        assert!(glob_match("axi_*_i", "axi_awvalid_i"));
        assert!(glob_match("?clk_i", "pclk_i"));
        assert!(!glob_match("?clk_i", "axi_aclk_i"));
    }

    #[test]
    fn a_pattern_with_no_metacharacter_is_an_exact_match() {
        assert!(glob_match("clk_i", "clk_i"));
        assert!(!glob_match("clk_i", "clk_ii"));
        assert!(!glob_match("clk_i", "xclk_i"));
    }

    #[test]
    fn backtracking_handles_more_than_one_star() {
        assert!(glob_match("a*b*c", "axxbyyc"));
        assert!(!glob_match("a*b*c", "axxbyy"));
        assert!(glob_match("*", "anything"));
    }

    #[test]
    fn a_bus_wide_input_delay_no_longer_reports_a_missing_port() {
        let n = crate::netlist::parse(
            "module t(clk,paddr_i,dout);\ninput clk; input [1:0] paddr_i; output dout;\n\
             DFF r(.CK(clk),.D(paddr_i[0]),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [get_ports {paddr_i[*]}]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&n, &sdc, &Lib::load("tests/data/sdc_lint/seq.lib").unwrap());
        assert!(
            !r.findings.iter().any(|f| f.code == "delay-unknown-port"),
            "a bus wildcard names real ports: {:?}",
            r.findings
        );
        assert!(
            !r.findings.iter().any(|f| f.code == "unconstrained-input"),
            "and it constrains them: {:?}",
            r.findings
        );
    }
}

#[cfg(test)]
mod declared_domain_tests {
    use super::*;
    use crate::ipmeta::IpMeta;

    fn lib() -> Lib {
        Lib::load("tests/data/sdc_lint/seq.lib").expect("seq.lib")
    }
    fn nl() -> Netlist {
        crate::netlist::parse(
            "module t(clk,pclk,din,dout);\ninput clk,pclk,din; output dout;\n\
             DFF r(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }
    const TWO_DOMAINS: &str = r#"{"clock_domains":[
        {"name":"core","clock":"clk"},{"name":"apb","clock":"pclk"}],
        "clock_domain_relations":{"asynchronous_groups":[["core"],["apb"]]}}"#;

    #[test]
    fn a_declared_domain_the_sdc_never_constrains_is_an_error() {
        // Exactly the fft_top defect: the IP has two clock domains, the flow config named one,
        // and timing passes while half the design is untimed. Only the declaration catches it.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        let f = r
            .findings
            .iter()
            .find(|f| f.code == "domain-not-constrained")
            .expect("the undeclared domain must be reported");
        assert_eq!(f.severity, Severity::Error);
        assert!(
            f.message.contains("apb") && f.message.contains("pclk"),
            "{}",
            f.message
        );
    }

    #[test]
    fn constraining_every_declared_domain_is_quiet() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             create_clock -name pclk -period 10 [get_ports pclk]\n\
             set_clock_groups -asynchronous -group {clk} -group {pclk}\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        assert!(
            !r.findings.iter().any(
                |f| f.code == "domain-not-constrained" || f.code == "async-groups-not-declared"
            ),
            "a correct SDC must stay quiet: {:?}",
            r.findings
        );
    }

    #[test]
    fn declared_async_domains_with_no_clock_groups_warn_but_do_not_error() {
        // Both domains constrained, but the SDC does not say they are unrelated — the paths
        // between them get timed against an edge relation that does not exist. Pessimistic,
        // not absent, so it is a warning.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             create_clock -name pclk -period 10 [get_ports pclk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        let f = r
            .findings
            .iter()
            .find(|f| f.code == "async-groups-not-declared")
            .expect("reported");
        assert_eq!(f.severity, Severity::Warning);
        assert!(!r
            .findings
            .iter()
            .any(|f| f.code == "domain-not-constrained"));
    }

    #[test]
    fn without_metadata_the_check_does_not_run_at_all() {
        // Most IPs carry no clock_domains yet. Absence of a declaration is not a finding.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(!r
            .findings
            .iter()
            .any(|f| f.code == "domain-not-constrained"));
    }
}

#[cfg(test)]
mod hierarchical_name_tests {
    use super::*;

    /// A flattened block's register is an instance whose own name has a `/` (`u_b/r`): an
    /// exception to it is live, as it is for the same register in a flat netlist.
    #[test]
    fn an_exception_on_a_flattened_register_is_not_dead() {
        let lib = Lib::load("tests/data/sdc_lint/seq.lib").expect("seq.lib");
        let nl = crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\nwire mid;\n\
             DFF u_a/r(.CK(clk),.D(din),.Q(mid));\nDFF u_b/r(.CK(clk),.D(mid),.Q(dout));\nendmodule\n"
                .replace("u_a/r", "\\u_a/r ")
                .replace("u_b/r", "\\u_b/r ")
                .as_str(),
        )
        .expect("netlist");
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\nset_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\nset_false_path -to [get_pins u_b/r/D]\n\
             set_false_path -from [get_pins u_a/r/CK] -to [get_pins u_b/r/D]\n",
        )
        .expect("sdc");
        let r = lint(&nl, &sdc, &lib);
        // the one exception finding is the overlap check's: the -from/-to false path is inside
        // the -to one, which already cuts every path it names
        let exc: Vec<&Finding> = r.findings.iter().filter(|f| f.code.starts_with("exception-")).collect();
        assert_eq!(exc.len(), 1, "{:?}", r.findings);
        assert_eq!(exc[0].code, "exception-redundant", "{:?}", exc);
        assert!(exc[0].message.starts_with("`set_false_path -from"), "{}", exc[0].message);
    }
}

#[cfg(test)]
mod braced_variable_tests {
    use super::*;

    /// OpenTitan's form: the clock port in braces is not substituted, so it is never removed;
    /// the same name written without braces is the variable's value and is not reported.
    #[test]
    fn a_variable_left_in_braces_is_a_warning() {
        let s = Sdc::parse(
            "set P clk_i\ncreate_clock -name clk_i -period 10 [get_ports clk_i]\n\
             set_input_delay 1 [remove_from_collection [all_inputs] {${P}}] -clock clk_i\n\
             set_input_delay 2 [remove_from_collection [all_inputs] $P] -clock clk_i\n",
        )
        .unwrap();
        let r = lint_without_design(&s);
        let w: Vec<&Finding> = r.findings.iter().filter(|f| f.code == "braced-variable").collect();
        assert_eq!(w.len(), 1, "{:?}", r.findings);
        assert!(w[0].message.contains("${P}") && w[0].severity == Severity::Warning);
    }
}
