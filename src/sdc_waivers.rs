//! Waivers for constraint findings, as a Tcl file (SDC scope S3).
//!
//! A waiver file is plain Tcl, one command per waiver:
//!
//! ```tcl
//! waive_finding -id exception-shadowed:3f2a9c1d07be -code exception-shadowed \
//!     -objects {{set_multicycle_path 3 -from [get_pins r1/CK]}} \
//!     -reason {reset release is checked by the reset-synchronizer review} \
//!     -owner {jdoe} -expires 2027-06-30
//! ```
//!
//! - The file is meant to travel with the Tcl scripts a flow already sources: another tool can
//!   `source` it after defining `waive_finding`, or read it as data. Our reader parses only this
//!   command: commands are split as the SDC reader splits them, and words by Tcl's rules for
//!   braces, quotes and backslashes (a backslash-escaped brace does not nest). The file needs no
//!   other Tcl.
//! - **The id is the finding's identity**: its code and its message, hashed. Messages name what a
//!   finding is about (the objects, the commands), never a line number or a position in the file,
//!   so an id survives edits elsewhere in the SDC and changes when what the finding is about
//!   changes. `-code` and `-objects` are for the reader of the file; matching is by `-id`.
//! - **Applying** sorts each finding into waived or active, and reports, as findings of their own:
//!   a waiver that matches nothing (`waiver-stale`), one with no reason (`waiver-no-reason`, and
//!   it waives nothing), one past its `-expires` date (`waiver-expired`, same), the same id twice
//!   (`waiver-duplicate`), and a command the reader does not accept (`waiver-malformed`).
//! - The approval of a waiver (who decided, when, in which review) is recorded outside this file
//!   by whoever runs the review; nothing here depends on it.

use crate::sdc::logical_lines;
use crate::sdc_lint::{Finding, LintReport, Severity};

/// One `waive_finding` command.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Waiver {
    pub id: String,
    pub code: String,
    /// What the finding is about, as the template wrote it: the spans the message quotes.
    pub objects: Vec<String>,
    pub reason: String,
    pub owner: Option<String>,
    /// `YYYY-MM-DD`; the waiver applies through that day.
    pub expires: Option<String>,
    /// The command's position in the waiver file, from 1.
    pub index: usize,
}

/// A waiver file read: its waivers, and each command that was not a valid waiver, with why.
#[derive(Debug, Clone, Default)]
pub struct WaiverFile {
    pub waivers: Vec<Waiver>,
    pub malformed: Vec<String>,
}

/// The findings after the waivers: those still active (with the waiver findings added), and
/// those waived, each with the waiver that waived it.
#[derive(Debug, Default)]
pub struct Applied {
    pub active: LintReport,
    pub waived: Vec<(Finding, Waiver)>,
}

/// FNV-1a, 64 bits: a fixed, documented hash, so an id is the same on every machine and in every
/// release (the standard library's hasher promises neither).
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// A finding's stable id: `<code>:<12 hex digits>`, from its code and its message.
#[must_use]
pub fn finding_id(f: &Finding) -> String {
    let mut key = f.code.as_bytes().to_vec();
    key.push(0);
    key.extend_from_slice(f.message.as_bytes());
    format!("{}:{:012x}", f.code, fnv1a(&key) >> 16)
}

/// The spans a message quotes in backticks: what the finding is about, for `-objects`.
fn quoted(message: &str) -> Vec<String> {
    message.split('`').skip(1).step_by(2).map(str::to_string).collect()
}

/// A Tcl word for `s`, braced. A brace inside is escaped so the word still closes where it
/// should (Tcl keeps the backslash; our reader drops it).
fn brace(s: &str) -> String {
    // balanced as Tcl counts braces: an escaped one does not count
    let (mut depth, mut esc, mut ok) = (0i32, false, true);
    for c in s.chars() {
        match (esc, c) {
            (true, _) => esc = false,
            (false, '\\') => esc = true,
            (false, '{') => depth += 1,
            (false, '}') => {
                depth -= 1;
                ok &= depth >= 0;
            }
            _ => {}
        }
    }
    let balanced = ok && depth == 0 && !esc;
    if balanced {
        format!("{{{s}}}")
    } else {
        format!("{{{}}}", s.replace('\\', "\\\\").replace('{', "\\{").replace('}', "\\}"))
    }
}

/// A waiver template for every error and warning: one `waive_finding` with an empty `-reason`,
/// preceded by the finding as a comment. Information findings are not waivable: they count
/// toward nothing.
#[must_use]
pub fn write_template(r: &LintReport, header: &str) -> String {
    let mut t = String::new();
    for l in header.lines() {
        t += &format!("# {l}\n");
    }
    t += "# Fill in -reason for each finding you accept, and delete the ones you will fix.\n";
    t += "# A waiver with an empty -reason waives nothing.\n";
    for f in r.findings.iter().filter(|f| f.severity != Severity::Info) {
        let sev = if f.severity == Severity::Error { "error" } else { "warning" };
        t += "\n";
        for l in f.message.lines() {
            t += &format!("# {sev} {}: {l}\n", f.code);
        }
        let objs: Vec<String> = quoted(&f.message).iter().map(|o| brace(o)).collect();
        t += &format!("waive_finding -id {} -code {} -objects {} -reason {{}}\n", finding_id(f), f.code, brace(&objs.join(" ")));
    }
    t
}

/// One command's words, by Tcl's rules: a braced word runs to its matching brace (a
/// backslash-escaped brace does not count) and keeps its text as written; a quoted word runs to
/// the next unescaped quote; any other word runs to a blank. Brackets are not run: they are text.
fn words(cmd: &str) -> Vec<String> {
    let cs: Vec<char> = cmd.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < cs.len() {
        if cs[i].is_whitespace() {
            i += 1;
            continue;
        }
        let mut w = String::new();
        match cs[i] {
            '{' => {
                let mut depth = 1;
                i += 1;
                while i < cs.len() {
                    match cs[i] {
                        '\\' if i + 1 < cs.len() => {
                            w.push(cs[i]);
                            w.push(cs[i + 1]);
                            i += 2;
                            continue;
                        }
                        '{' => depth += 1,
                        '}' => {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                        _ => {}
                    }
                    w.push(cs[i]);
                    i += 1;
                }
                i += 1;
            }
            '"' => {
                i += 1;
                while i < cs.len() && cs[i] != '"' {
                    if cs[i] == '\\' && i + 1 < cs.len() {
                        w.push(cs[i]);
                        i += 1;
                    }
                    w.push(cs[i]);
                    i += 1;
                }
                i += 1;
            }
            _ => {
                while i < cs.len() && !cs[i].is_whitespace() {
                    w.push(cs[i]);
                    i += 1;
                }
            }
        }
        out.push(w);
    }
    out
}

/// Undo the escapes [`brace`] writes.
fn word(t: &str) -> String {
    let mut out = String::new();
    let mut cs = t.chars().peekable();
    while let Some(c) = cs.next() {
        match (c, cs.peek()) {
            ('\\', Some('{' | '}' | '\\')) => out.extend(cs.next()),
            _ => out.push(c),
        }
    }
    out
}

/// The elements of a Tcl list, each as [`words`] reads it.
fn list(t: &str) -> Vec<String> {
    words(t).iter().map(|x| word(x)).collect()
}

/// Read a waiver file. A command that is not a valid `waive_finding` is listed in
/// [`WaiverFile::malformed`] and the rest are still read.
#[must_use]
pub fn parse(text: &str) -> WaiverFile {
    let mut out = WaiverFile::default();
    for (k, cmd) in logical_lines(text).iter().enumerate() {
        let index = k + 1;
        let toks = words(cmd);
        let Some(head) = toks.first() else { continue };
        if head != "waive_finding" {
            out.malformed.push(format!("command {index}: `{head}` is not `waive_finding`"));
            continue;
        }
        let mut w = Waiver { index, ..Waiver::default() };
        let mut bad = None;
        let mut i = 1;
        while i < toks.len() {
            let Some(v) = toks.get(i + 1) else {
                bad = Some(format!("`{}` has no value", toks[i]));
                break;
            };
            match toks[i].as_str() {
                "-id" => w.id = word(v),
                "-code" => w.code = word(v),
                "-objects" => w.objects = list(v),
                "-reason" => w.reason = word(v),
                "-owner" => w.owner = Some(word(v)),
                "-expires" => w.expires = Some(word(v)),
                other => {
                    bad = Some(format!("`{other}` is not an option of `waive_finding`"));
                    break;
                }
            }
            i += 2;
        }
        if bad.is_none() && w.id.is_empty() {
            bad = Some("no `-id`".into());
        }
        if bad.is_none() {
            if let Some(e) = w.expires.as_deref().filter(|e| !is_date(e)) {
                bad = Some(format!("`-expires {e}` is not a YYYY-MM-DD date"));
            }
        }
        match bad {
            Some(why) => out.malformed.push(format!("command {index} ({}): {why}", if w.id.is_empty() { "no id" } else { &w.id })),
            None => out.waivers.push(w),
        }
    }
    out
}

fn is_date(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 10 && b[4] == b'-' && b[7] == b'-' && b.iter().enumerate().all(|(i, c)| i == 4 || i == 7 || c.is_ascii_digit())
}

/// Apply `file` to `r` on day `today` (`YYYY-MM-DD`, UTC). A waiver applies when its id is a
/// finding's id, it has a reason, and `today` is not after its `-expires` date.
#[must_use]
pub fn apply(r: &LintReport, file: &WaiverFile, today: &str) -> Applied {
    let mut extra: Vec<Finding> = Vec::new();
    let warn = |code: &'static str, message: String| Finding { severity: Severity::Warning, code, message };
    for m in &file.malformed {
        extra.push(warn("waiver-malformed", format!("waiver file {m}; it waives nothing")));
    }
    // the first waiver of an id is the one used; a later one is reported
    let mut usable: Vec<&Waiver> = Vec::new();
    for w in &file.waivers {
        if usable.iter().any(|u| u.id == w.id) {
            extra.push(warn("waiver-duplicate", format!("waiver `{}` (command {}) repeats an earlier waiver of the same id; only the first is used", w.id, w.index)));
        } else {
            usable.push(w);
        }
    }
    let ids: Vec<String> = r.findings.iter().map(finding_id).collect();
    let mut out = Applied::default();
    for (f, id) in r.findings.iter().zip(&ids) {
        match usable.iter().find(|w| w.id == *id) {
            Some(w) if w.reason.trim().is_empty() => {
                extra.push(warn("waiver-no-reason", format!("waiver `{}` has no -reason, so it waives nothing", w.id)));
                out.active.findings.push(f.clone());
            }
            Some(w) if w.expires.as_deref().is_some_and(|e| today > e) => {
                extra.push(warn("waiver-expired", format!("waiver `{}` expired on {}, so it waives nothing", w.id, w.expires.as_deref().unwrap_or(""))));
                out.active.findings.push(f.clone());
            }
            Some(w) => out.waived.push((f.clone(), (*w).clone())),
            None => out.active.findings.push(f.clone()),
        }
    }
    for w in &usable {
        if !ids.contains(&w.id) {
            extra.push(warn("waiver-stale", format!("waiver `{}` (command {}) matches no finding: what it waived was fixed or has changed", w.id, w.index)));
        }
    }
    out.active.findings.extend(extra);
    out.active.findings.sort_by(|a, b| (a.severity as u8, a.code).cmp(&(b.severity as u8, b.code)));
    out
}

/// How to write our waivers as another tool's Tcl waiver commands: an export template.
///
/// A template is a small text file, one directive per line (`#` starts a comment):
///
/// ```text
/// command  my_waive -rule {@rule@} -match {@objects@} -comment {@reason@}[[ -owner {@owner@}]]
/// object   name=~"@object@"
/// join     \s||\s
/// rule     exception-shadowed   OVERLAP_SHADOW
/// rule     exception-conflict   OVERLAP_CONFLICT OVERLAP_VALUE
/// ```
///
/// - `command`: the line written per waiver. Placeholders: `@rule@` (the target's rule ids,
///   space-separated), `@objects@`, `@reason@`, `@owner@`, `@expires@`, `@id@`, `@code@`. A part in
///   `[[…]]` is written only when every placeholder in it has a value.
/// - `object`: how one of the waiver's `-objects` is written (`@object@`); `join` goes between
///   them (`\s`, `\t`, `\n` are a blank, a tab, a newline). Defaults: `@object@` and one blank.
/// - `rule <our code> <their rule ids…>`: the mapping. `*` maps every code not mapped otherwise,
///   and `@code@` in it is our own code. ⛔ A waiver whose code has no mapping is NOT written: it
///   is listed as skipped. A guessed rule id would be a waiver the target silently ignores.
///
/// Every value is inserted as written, except that a value whose braces do not balance has them
/// escaped, so the command still parses as Tcl.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportTemplate {
    pub command: String,
    pub object: String,
    pub join: String,
    pub rules: Vec<(String, Vec<String>)>,
}

const PLACEHOLDERS: [&str; 7] = ["rule", "objects", "reason", "owner", "expires", "id", "code"];

/// The `@name@` placeholders in `t`.
fn placeholders(t: &str) -> Vec<String> {
    let parts: Vec<&str> = t.split('@').collect();
    let mut out = Vec::new();
    // an odd-indexed part between two `@` is a candidate name
    for (k, p) in parts.iter().enumerate() {
        if k % 2 == 1 && k + 1 < parts.len() && !p.is_empty() && p.chars().all(|c| c.is_ascii_lowercase() || c == '_') {
            out.push(p.to_string());
        }
    }
    out
}

/// Read an export template. An unknown directive or placeholder is an error: a typo must not
/// become text in another tool's waiver file.
pub fn parse_template(text: &str) -> Result<ExportTemplate, String> {
    let mut t = ExportTemplate { command: String::new(), object: "@object@".into(), join: " ".into(), rules: Vec::new() };
    for (n, raw) in text.lines().enumerate() {
        let line = raw.trim_start();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (key, rest) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        let rest = rest.trim_start();
        match key {
            "command" => t.command = rest.trim_end().to_string(),
            "object" => t.object = rest.trim_end().to_string(),
            "join" => t.join = rest.trim_end().replace("\\s", " ").replace("\\t", "\t").replace("\\n", "\n"),
            "rule" => {
                let mut w = rest.split_whitespace();
                let code = w.next().ok_or(format!("line {}: `rule` needs our code and the target's rule ids", n + 1))?;
                let ids: Vec<String> = w.map(str::to_string).collect();
                if ids.is_empty() {
                    return Err(format!("line {}: `rule {code}` names no rule id of the target", n + 1));
                }
                t.rules.push((code.to_string(), ids));
            }
            other => return Err(format!("line {}: `{other}` is not a template directive (command, object, join, rule)", n + 1)),
        }
    }
    if t.command.is_empty() {
        return Err("the template has no `command` line".into());
    }
    for p in placeholders(&t.command) {
        if !PLACEHOLDERS.contains(&p.as_str()) {
            return Err(format!("`@{p}@` in `command` is not a placeholder ({})", PLACEHOLDERS.join(", ")));
        }
    }
    for p in placeholders(&t.object) {
        if p != "object" {
            return Err(format!("`@{p}@` in `object` is not a placeholder (only @object@)"));
        }
    }
    Ok(t)
}

/// A value as inserted: its braces escaped when they do not balance.
fn tcl_safe(v: &str) -> String {
    let b = brace(v);
    b[1..b.len() - 1].to_string()
}

/// `[[…]]` parts kept when all their placeholders have values, dropped otherwise; then every
/// placeholder replaced.
fn render(t: &str, vals: &[(&str, String)]) -> String {
    let get = |k: &str| vals.iter().find(|(n, _)| *n == k).map(|(_, v)| v.as_str()).unwrap_or("");
    let mut out = String::new();
    let mut rest = t;
    while let Some(a) = rest.find("[[") {
        let Some(b) = rest[a..].find("]]").map(|b| a + b) else { break };
        out += &rest[..a];
        let part = &rest[a + 2..b];
        if placeholders(part).iter().all(|p| !get(p).is_empty()) {
            out += part;
        }
        rest = &rest[b + 2..];
    }
    out += rest;
    for (k, v) in vals {
        out = out.replace(&format!("@{k}@"), &tcl_safe(v));
    }
    out
}

/// Our waivers written as the template's commands, and those not written, with why.
#[derive(Debug, Clone, Default)]
pub struct Exported {
    pub text: String,
    pub skipped: Vec<String>,
}

/// Write `file`'s waivers with `t`. The header lines are written as `#` comments first.
#[must_use]
pub fn export(file: &WaiverFile, t: &ExportTemplate, header: &str) -> Exported {
    let mut out = Exported::default();
    for l in header.lines() {
        out.text += &format!("# {l}\n");
    }
    for w in &file.waivers {
        let ids = t.rules.iter().find(|(c, _)| *c == w.code).or_else(|| t.rules.iter().find(|(c, _)| c == "*"));
        let Some((_, ids)) = ids else {
            out.skipped.push(format!("`{}`: no rule of the target is mapped to `{}`", w.id, w.code));
            continue;
        };
        if w.reason.trim().is_empty() {
            out.skipped.push(format!("`{}`: no -reason", w.id));
            continue;
        }
        let rule = ids.iter().map(|r| r.replace("@code@", &w.code)).collect::<Vec<_>>().join(" ");
        let objects = w.objects.iter().map(|o| t.object.replace("@object@", &tcl_safe(o))).collect::<Vec<_>>().join(&t.join);
        let vals = [
            ("rule", rule),
            ("reason", w.reason.clone()),
            ("owner", w.owner.clone().unwrap_or_default()),
            ("expires", w.expires.clone().unwrap_or_default()),
            ("id", w.id.clone()),
            ("code", w.code.clone()),
        ];
        // objects last and unescaped again: each was made safe as it was rendered
        let line = render(&t.command.replace("@objects@", "\u{0}OBJECTS\u{0}"), &vals).replace("\u{0}OBJECTS\u{0}", &objects);
        out.text += &line;
        out.text += "\n";
    }
    out
}

/// Today's date in UTC, `YYYY-MM-DD`: what `-expires` is compared with.
#[must_use]
pub fn utc_today() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    civil(i64::try_from(secs / 86_400).unwrap_or(0))
}

/// The civil date of a day count since 1970-01-01 (the proleptic Gregorian calendar's
/// days-to-civil conversion).
fn civil(days: i64) -> String {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdc::Sdc;
    use crate::sdc_lint::lint_without_design;

    const SDC: &str = "create_clock -name clk -period 10 [get_ports clk]\n\
        set_false_path -from [get_pins r1/CK]\n\
        set_multicycle_path 3 -from [get_pins r1/CK]\n\
        set_multicycle_path 2 -through [get_pins u1/Y]\n\
        set_multicycle_path 4 -through [get_pins u1/Y]\n\
        create_clock -name clk -period 5 [get_ports clk2]\n";

    fn report(text: &str) -> LintReport {
        lint_without_design(&Sdc::parse(text).expect("parse"))
    }
    fn waivable(r: &LintReport) -> Vec<&Finding> {
        r.findings.iter().filter(|f| f.severity != Severity::Info).collect()
    }
    /// The template with every `-reason {}` filled in.
    fn accepted(r: &LintReport) -> String {
        write_template(r, "test").replace("-reason {}", "-reason {accepted in review}")
    }

    #[test]
    fn reordering_the_file_keeps_every_id() {
        let mut lines: Vec<&str> = SDC.lines().collect();
        let a: Vec<String> = waivable(&report(SDC)).iter().map(|f| finding_id(f)).collect();
        // the two equal multicycles keep their order (which one is later is what the finding says)
        lines.swap(0, 5);
        lines.swap(1, 2);
        let b: Vec<String> = waivable(&report(&(lines.join("\n") + "\n"))).iter().map(|f| finding_id(f)).collect();
        assert!(a.len() >= 3, "the fixture must have findings to waive: {a:?}");
        let (mut a, mut b) = (a, b);
        a.sort();
        b.sort();
        assert_eq!(a, b);
    }

    #[test]
    fn changing_an_object_changes_the_id() {
        let a: Vec<String> = waivable(&report(SDC)).iter().map(|f| finding_id(f)).collect();
        let b: Vec<String> = waivable(&report(&SDC.replace("r1/CK", "r9/CK"))).iter().map(|f| finding_id(f)).collect();
        let moved = a.iter().filter(|x| !b.contains(x)).count();
        assert_eq!(moved, 1, "only the finding about r1/CK changes: {a:?} vs {b:?}");
    }

    #[test]
    fn writing_then_reading_the_waivers_leaves_nothing_active_and_one_removed_comes_back() {
        let r = report(SDC);
        let n = waivable(&r).len();
        let file = parse(&accepted(&r));
        assert!(file.malformed.is_empty(), "{:?}", file.malformed);
        assert_eq!(file.waivers.len(), n);
        let all = apply(&r, &file, "2026-11-07");
        assert_eq!(all.waived.len(), n);
        assert!(waivable(&all.active).is_empty(), "{:?}", all.active.findings);
        for k in 0..n {
            let mut less = file.clone();
            let gone = less.waivers.remove(k);
            let some = apply(&r, &less, "2026-11-07");
            let back: Vec<String> = waivable(&some.active).iter().map(|f| finding_id(f)).collect();
            assert_eq!(back, vec![gone.id.clone()], "removing waiver {k} brings back exactly its finding");
        }
    }

    #[test]
    fn a_waiver_with_no_reason_or_past_its_date_waives_nothing_and_says_so() {
        let r = report(SDC);
        let t = write_template(&r, "test");
        let a = apply(&r, &parse(&t), "2026-11-07");
        assert!(a.waived.is_empty());
        assert!(a.active.findings.iter().any(|f| f.code == "waiver-no-reason"));
        let dated = accepted(&r).replace("-reason {accepted in review}", "-reason {ok} -expires 2026-11-06");
        let a = apply(&r, &parse(&dated), "2026-11-07");
        assert!(a.waived.is_empty());
        assert!(a.active.findings.iter().any(|f| f.code == "waiver-expired"));
        // through the date itself it applies
        let a = apply(&r, &parse(&dated), "2026-11-06");
        assert_eq!(a.waived.len(), waivable(&r).len());
    }

    #[test]
    fn a_stale_duplicate_or_malformed_waiver_is_a_finding() {
        let r = report(SDC);
        let mut t = accepted(&r);
        t += "waive_finding -id exception-shadowed:000000000000 -code exception-shadowed -reason {gone}\n";
        let first = parse(&t).waivers[0].clone();
        t += &format!("waive_finding -id {} -reason {{again}}\n", first.id);
        t += "waive_finding -id x -reason {r} -severity low\n";
        t += "set_false_path -from [get_ports a]\n";
        t += "waive_finding -id y -reason {r} -expires tomorrow\n";
        let a = apply(&r, &parse(&t), "2026-11-07");
        let codes: Vec<&str> = a.active.findings.iter().map(|f| f.code).collect();
        assert_eq!(codes.iter().filter(|c| **c == "waiver-stale").count(), 1, "{codes:?}");
        assert_eq!(codes.iter().filter(|c| **c == "waiver-duplicate").count(), 1, "{codes:?}");
        assert_eq!(codes.iter().filter(|c| **c == "waiver-malformed").count(), 3, "{codes:?}");
        assert_eq!(a.waived.len(), waivable(&r).len(), "the valid waivers still apply");
    }

    /// The file is Tcl another tool can source: braces balance in every command, including a
    /// message that quotes an unbalanced brace.
    #[test]
    fn the_template_is_tcl_with_balanced_words() {
        let f = Finding { severity: Severity::Warning, code: "sdc-x", message: "odd `a{b` and `c}` here".into() };
        let r = LintReport { findings: vec![f.clone()] };
        let t = write_template(&r, "h");
        let file = parse(&t);
        assert!(file.malformed.is_empty(), "{:?}\n{t}", file.malformed);
        assert_eq!(file.waivers[0].objects, vec!["a{b".to_string(), "c}".to_string()], "{t}");
        assert_eq!(file.waivers[0].id, finding_id(&f));
        let cmd = t.lines().find(|l| l.starts_with("waive_finding")).expect("a command");
        let depth = cmd.chars().fold((0i32, false), |(d, esc), c| match (esc, c) {
            (true, _) => (d, false),
            (false, '\\') => (d, true),
            (false, '{') => (d + 1, false),
            (false, '}') => (d - 1, false),
            _ => (d, false),
        });
        assert_eq!(depth.0, 0, "{cmd}");
    }

    /// Our own command as a template: exporting with it and reading the result back gives the
    /// same waivers (the export round-trips through its own reader).
    const SELF: &str = "# our own format\n\
        command waive_finding -id @id@ -code @code@ -objects {@objects@} -reason {@reason@}[[ -owner {@owner@}]][[ -expires @expires@]]\n\
        object {@object@}\n\
        rule * @code@\n";

    #[test]
    fn an_export_with_our_own_template_reads_back_as_the_same_waivers() {
        let r = report(SDC);
        let filled = accepted(&r).replacen("-reason {accepted in review}", "-reason {first {one}} -owner {qa team} -expires 2027-01-31", 1);
        let file = parse(&filled);
        let t = parse_template(SELF).expect("template");
        let e = export(&file, &t, "exported");
        assert!(e.skipped.is_empty(), "{:?}", e.skipped);
        let back = parse(&e.text);
        assert!(back.malformed.is_empty(), "{:?}\n{}", back.malformed, e.text);
        let strip = |w: &Waiver| Waiver { index: 0, ..w.clone() };
        assert_eq!(back.waivers.iter().map(strip).collect::<Vec<_>>(), file.waivers.iter().map(strip).collect::<Vec<_>>(), "{}", e.text);
        assert_eq!(apply(&r, &back, "2026-11-07").waived.len(), waivable(&r).len());
    }

    /// Golden text for a template in another tool's form: rule ids mapped per code, objects as
    /// match expressions, an optional part dropped when its value is absent, and an unmapped code
    /// skipped with a reason, never written with a guessed rule.
    #[test]
    fn an_export_in_another_form_maps_rules_and_skips_what_it_cannot_map() {
        let t = parse_template(
            "command my_waive -status {Waived} -rule {@rule@} -match {@objects@} -comment {@reason@}[[ -owner {@owner@}]]\n\
             object name=~\"@object@\"\n\
             join \\s||\\s\n\
             rule exception-shadowed OVERLAP_SHADOW\n\
             rule exception-conflict OVERLAP_CONFLICT OVERLAP_VALUE\n",
        )
        .expect("template");
        let file = parse(
            "waive_finding -id exception-shadowed:aaaaaaaaaaaa -code exception-shadowed -objects {{a/b} {c d}} -reason {intended} -owner {qa}\n\
             waive_finding -id exception-conflict:bbbbbbbbbbbb -code exception-conflict -objects {{x}} -reason {later one wins}\n\
             waive_finding -id clock-period:cccccccccccc -code clock-period -objects {} -reason {r}\n",
        );
        let e = export(&file, &t, "h");
        assert_eq!(
            e.text,
            "# h\n\
             my_waive -status {Waived} -rule {OVERLAP_SHADOW} -match {name=~\"a/b\" || name=~\"c d\"} -comment {intended} -owner {qa}\n\
             my_waive -status {Waived} -rule {OVERLAP_CONFLICT OVERLAP_VALUE} -match {name=~\"x\"} -comment {later one wins}\n"
        );
        assert_eq!(e.skipped.len(), 1, "{:?}", e.skipped);
        assert!(e.skipped[0].contains("clock-period"), "{:?}", e.skipped);
    }

    #[test]
    fn a_template_typo_is_an_error_not_text() {
        assert!(parse_template("command w -r {@rul@}\nrule * X\n").is_err());
        assert!(parse_template("comand w\n").is_err());
        assert!(parse_template("command w\nrule exception-shadowed\n").is_err());
        assert!(parse_template("object @object@\n").is_err(), "no command line");
        assert!(parse_template("command w -o {@objects@}\nobject @thing@\n").is_err());
    }

    #[test]
    fn a_value_with_unbalanced_braces_keeps_the_command_parseable() {
        let t = parse_template(SELF).expect("template");
        let file = parse("waive_finding -id x:1 -code x -objects {} -reason {a \\} b}\n");
        assert_eq!(file.waivers[0].reason, "a } b");
        let e = export(&file, &t, "");
        let back = parse(&e.text);
        assert!(back.malformed.is_empty(), "{:?}\n{}", back.malformed, e.text);
        assert_eq!(back.waivers[0].reason, "a } b", "{}", e.text);
    }

    #[test]
    fn the_civil_date_of_a_day_count() {
        assert_eq!(civil(0), "1970-01-01");
        assert_eq!(civil(19_723), "2024-01-01");
        assert_eq!(civil(19_782), "2024-02-29");
        assert_eq!(civil(19_783), "2024-03-01");
        assert_eq!(civil(-1), "1969-12-31");
        assert!(is_date(&utc_today()));
    }

    #[test]
    fn an_id_is_fixed_across_machines_and_releases() {
        // FNV-1a of a fixed key: a change here invalidates every waiver file users have written
        let f = Finding { severity: Severity::Warning, code: "clock-period", message: "clock `c` has a non-positive period (-1 ns)".into() };
        assert_eq!(finding_id(&f), format!("clock-period:{:012x}", fnv1a(b"clock-period\0clock `c` has a non-positive period (-1 ns)") >> 16));
        assert_eq!(fnv1a(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a(b"a"), 0xaf63_dc4c_8601_ec8c);
    }
}
