//! **`vyges loom fst`** — query an FST waveform in place: its header, its hierarchy, the
//! transitions of chosen signals in a time window, or their values at chosen times.
//!
//! ⛔ **Never convert an FST to VCD to inspect it.** The VCD of a simulation is many times the
//! size of its FST: it can fill a disk, and read into an agent's context it exhausts that too.
//! Ask the FST: `--info` reads only the header, `--list` the hierarchy, and a transition query
//! reads only the blocks its window touches, printing at most `--limit` lines (1000 by
//! default) and saying so on stderr when it stops.
//!
//! ## The contract (the same as CoralNPU's `fst_query`, which is the oracle it is gated on)
//!
//! - `--info`: file, writer, date, timescale, time range, scope and signal counts.
//! - `--list [--scope S] [--depth N] [--scopes_only]`: `name [width] (handle=h)` per signal, or
//!   one scope path per line.
//! - transitions: `@<time> <name> = <value>`, ordered by time then handle, an alias printed
//!   under each of its names; `--start`/`--end` (inclusive) bound the window; `--limit N`
//!   (0 = no limit), `--tail N` (the last N), `--count` (`N transitions`).
//! - `--sample_at T1,T2,…`: `@<time> (sample) <name> = <value>`, the value in effect at each
//!   time (a change AT the time counts), `x` for a signal with no value yet.
//! - patterns: substring by default, `--exact` for whole names, `--regex` (whole name with
//!   `--exact`, anywhere otherwise). Names are the dotted scope path and the variable name
//!   without the range a writer may append.
//! - values: 2 to 64 bits all `0`/`1` print as hex (`0x1f`); one bit, a wider or non-binary
//!   value prints as its levels.
//! - `--batch FILE|-`: one query per line, `#` comments, `exit`/`quit`; each query's output
//!   ends with `--- [query end] ---`.
//!
//! What this adds to that contract: real and string signals are decoded (the oracle skips
//! them — a real prints as `%.16g`, a string as its text), and `--json` output.
//!
//! The first block's frame stands for the initial state, as the oracle reads it: a signal with
//! no change at the first block's start time gets its frame value there.

#![deny(clippy::unwrap_used)]
#![cfg_attr(test, allow(clippy::unwrap_used))]

use std::collections::{BTreeMap, HashSet, VecDeque};
use std::io::Write;

use crate::fst::FstError;
use crate::fst_wave::{fmt_g16, vt, FstReader, Hier, Value};

/// Signal-name matching: substring, exact, or regular expression.
pub struct Matcher {
    patterns: Vec<String>,
    regexes: Vec<regex::Regex>,
    exact: bool,
    use_regex: bool,
}

impl Matcher {
    pub fn new(patterns: &[String], use_regex: bool, exact: bool) -> Result<Matcher, FstError> {
        let mut regexes = Vec::new();
        if use_regex {
            for p in patterns {
                let src = if exact { format!("^(?:{p})$") } else { p.clone() };
                match regex::Regex::new(&src) {
                    Ok(r) => regexes.push(r),
                    Err(e) => return Err(FstError(format!("Invalid regular expression '{p}': {e}"))),
                }
            }
        }
        Ok(Matcher { patterns: patterns.to_vec(), regexes, exact, use_regex })
    }

    pub fn matches(&self, name: &str) -> bool {
        if self.patterns.is_empty() {
            return true;
        }
        if self.use_regex {
            return self.regexes.iter().any(|r| r.is_match(name));
        }
        if self.exact {
            return self.patterns.iter().any(|p| p == name);
        }
        self.patterns.iter().any(|p| name.contains(p.as_str()))
    }
}

/// One query, as the flags give it.
#[derive(Debug, Clone, PartialEq)]
pub struct QueryOptions {
    pub info: bool,
    pub list: bool,
    pub start: u64,
    pub end: u64,
    pub limit: u64,
    pub tail: u64,
    pub count: bool,
    pub sample_at: String,
    pub scope: String,
    pub depth: u64,
    pub scopes_only: bool,
    pub regex: bool,
    pub exact: bool,
    pub json: bool,
    pub patterns: Vec<String>,
}

impl Default for QueryOptions {
    fn default() -> Self {
        QueryOptions {
            info: false,
            list: false,
            start: 0,
            end: u64::MAX,
            limit: 1000,
            tail: 0,
            count: false,
            sample_at: String::new(),
            scope: String::new(),
            depth: 0,
            scopes_only: false,
            regex: false,
            exact: false,
            json: false,
            patterns: Vec::new(),
        }
    }
}

/// What parsing a query's arguments gave.
pub enum Parsed {
    Query(QueryOptions, Option<String>),
    Help,
}

/// A declared signal, as a query sees it.
#[derive(Debug, Clone)]
pub struct Signal {
    pub handle: u32,
    pub name: String,
    pub width: u32,
    pub kind: u8,
}

impl Signal {
    fn is_real(&self) -> bool {
        vt::is_real(self.kind)
    }
}

/// An opened file with its names resolved, for any number of queries.
pub struct Session {
    pub path: String,
    pub reader: FstReader,
    pub signals: Vec<Signal>,
    pub scopes: Vec<String>,
    /// The first block's start and frame.
    initial: Option<(u64, Vec<u8>)>,
    /// Handles declared (not aliases).
    num_handles: usize,
}

/// `strtoull(.., 0)`-style: decimal, `0x` hex, leading-`0` octal; no sign.
fn parse_u64(s: &str) -> Option<u64> {
    let t = s.trim_start_matches([' ', '\t']);
    if t.is_empty() || t.starts_with('-') || t.starts_with('+') {
        return None;
    }
    if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        return u64::from_str_radix(h, 16).ok();
    }
    if t.len() > 1 && t.starts_with('0') {
        return u64::from_str_radix(&t[1..], 8).ok();
    }
    t.parse().ok()
}

fn parse_bool(v: &str) -> Option<bool> {
    match v {
        "true" | "1" | "t" | "yes" | "y" => Some(true),
        "false" | "0" | "f" | "no" | "n" => Some(false),
        _ => None,
    }
}

/// Parse a query's flags (`-f`, `--f`, `--f=v`, `--nof` for a switch) and patterns.
/// `allow_batch` is false inside a batch, where `--batch` is an error.
/// A parse failure has already been reported on `err`.
#[derive(Debug)]
pub struct ParseFailed;

pub fn parse_args(args: &[String], allow_batch: bool, err: &mut dyn Write) -> Result<Parsed, ParseFailed> {
    let mut o = QueryOptions::default();
    let mut batch = None;
    let mut positional_only = false;
    let mut i = 0usize;
    let mut fail = |m: String| {
        let _ = writeln!(err, "{m}");
    };
    while i < args.len() {
        let a = &args[i];
        i += 1;
        if positional_only || a.is_empty() || a == "-" || !a.starts_with('-') {
            o.patterns.push(a.clone());
            continue;
        }
        if a == "--" {
            positional_only = true;
            continue;
        }
        let s = a.strip_prefix("--").or_else(|| a.strip_prefix('-')).unwrap_or(a);
        let (name, val) = match s.split_once('=') {
            Some((n, v)) => (n, Some(v)),
            None => (s, None),
        };
        if name == "help" || name == "h" {
            return Ok(Parsed::Help);
        }
        let switches: [(&str, &mut bool); 7] = [
            ("info", &mut o.info),
            ("list", &mut o.list),
            ("count", &mut o.count),
            ("scopes_only", &mut o.scopes_only),
            ("regex", &mut o.regex),
            ("exact", &mut o.exact),
            ("json", &mut o.json),
        ];
        let mut handled = false;
        for (flag, target) in switches {
            if name == flag {
                match val {
                    None => *target = true,
                    Some(v) => match parse_bool(v) {
                        Some(b) => *target = b,
                        None => {
                            fail(format!("Invalid boolean value '{v}' for flag --{flag}"));
                            return Err(ParseFailed);
                        }
                    },
                }
                handled = true;
                break;
            }
            if val.is_none() && name.strip_prefix("no") == Some(flag) {
                *target = false;
                handled = true;
                break;
            }
        }
        if handled {
            continue;
        }
        let value = |i: &mut usize| -> Option<String> {
            if let Some(v) = val {
                return Some(v.to_string());
            }
            let v = args.get(*i).cloned();
            if v.is_some() {
                *i += 1;
            }
            v
        };
        match name {
            "start" | "end" | "limit" | "tail" | "depth" => {
                let Some(v) = value(&mut i) else {
                    fail(format!("Flag '--{name}' requires a value"));
                    return Err(ParseFailed);
                };
                let Some(n) = parse_u64(&v).filter(|&n| name != "depth" || n <= i32::MAX as u64) else {
                    fail(format!("Invalid value '{v}' for flag --{name}"));
                    return Err(ParseFailed);
                };
                match name {
                    "start" => o.start = n,
                    "end" => o.end = n,
                    "limit" => o.limit = n,
                    "tail" => o.tail = n,
                    _ => o.depth = n,
                }
            }
            "sample_at" | "scope" | "batch" => {
                let Some(v) = value(&mut i) else {
                    fail(format!("Flag '--{name}' requires a value"));
                    return Err(ParseFailed);
                };
                match name {
                    "sample_at" => o.sample_at = v,
                    "scope" => o.scope = v,
                    _ => {
                        if !allow_batch {
                            fail("Flag '--batch' is not supported inside a session or batch query".into());
                            return Err(ParseFailed);
                        }
                        batch = Some(v);
                    }
                }
            }
            _ => {
                fail(format!("Unknown flag: '{a}'"));
                return Err(ParseFailed);
            }
        }
    }
    Ok(Parsed::Query(o, batch))
}

/// Split a batch line into words: whitespace separates, single and double quotes group.
pub fn tokenize(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let (mut sq, mut dq, mut has) = (false, false, false);
    for c in line.chars() {
        if c == '\'' && !dq {
            sq = !sq;
            has = true;
        } else if c == '"' && !sq {
            dq = !dq;
            has = true;
        } else if c.is_whitespace() && !sq && !dq {
            if has {
                out.push(std::mem::take(&mut cur));
                has = false;
            }
        } else {
            cur.push(c);
            has = true;
        }
    }
    if has {
        out.push(cur);
    }
    out
}

fn time_unit(base: i32) -> &'static str {
    match base {
        -15 => "fs",
        -12 => "ps",
        -9 => "ns",
        -6 => "us",
        -3 => "ms",
        0 => "s",
        _ => "",
    }
}

/// (base power of ten, multiplier 1/10/100) of a timescale.
fn split_timescale(e: i8) -> (i32, u64) {
    let e = e as i32;
    let base = if e >= 0 { e / 3 * 3 } else { (e - 2) / 3 * 3 };
    let mult = match e - base {
        1 => 10,
        2 => 100,
        _ => 1,
    };
    (base, mult)
}

/// `1 ps (1e-12 s)`.
pub fn format_timescale(e: i8) -> String {
    let (base, mult) = split_timescale(e);
    let unit = time_unit(base);
    if unit.is_empty() {
        return format!("1e{e} s");
    }
    format!("{mult} {unit} (1e{e} s)")
}

/// How a value prints: 2..64 bits all 0/1 as hex, a real as `%.16g`, anything else as is.
fn value_text(v: &Value) -> String {
    match v {
        Value::Bits(b) => {
            let all_bin = !b.is_empty() && b.iter().all(|&c| c == b'0' || c == b'1');
            if all_bin && b.len() > 1 && b.len() <= 64 {
                let mut n = 0u64;
                for &c in b {
                    n = (n << 1) | (c - b'0') as u64;
                }
                format!("0x{n:x}")
            } else {
                String::from_utf8_lossy(b).into_owned()
            }
        }
        Value::Real(d) => fmt_g16(*d),
        Value::Str(s) => String::from_utf8_lossy(s).into_owned(),
    }
}

fn json_str(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn dots(s: &str) -> usize {
    s.bytes().filter(|&c| c == b'.').count()
}

impl Session {
    pub fn open(path: &str) -> Result<Session, FstError> {
        let mut reader = FstReader::open(path)?;
        let mut signals = Vec::new();
        let mut scopes = Vec::new();
        let mut seen = HashSet::new();
        let mut stack: Vec<String> = Vec::new();
        let mut num_handles = 0usize;
        for h in &reader.hier {
            match h {
                Hier::Scope { name, .. } => {
                    stack.push(name.clone());
                    let full = stack.join(".");
                    if seen.insert(full.clone()) {
                        scopes.push(full);
                    }
                }
                Hier::Upscope => {
                    stack.pop();
                }
                Hier::Var(v) => {
                    if !v.is_alias {
                        num_handles += 1;
                    }
                    let base = v.name.split(' ').next().unwrap_or(&v.name);
                    let mut name = String::new();
                    for s in &stack {
                        name.push_str(s);
                        name.push('.');
                    }
                    name.push_str(base);
                    signals.push(Signal { handle: v.handle, name, width: v.len, kind: v.kind });
                }
                _ => {}
            }
        }
        let initial = reader.initial_frame().unwrap_or(None);
        Ok(Session { path: path.to_string(), reader, signals, scopes, initial, num_handles })
    }

    fn frame_value(&self, h: u32) -> Option<Value> {
        let (_, frame) = self.initial.as_ref()?;
        if h == 0 || h as usize > self.num_handles {
            return None;
        }
        self.reader.frame_value(frame, h)
    }

    /// Run one query. Returns the exit status (0 = fine).
    pub fn run(&mut self, o: &QueryOptions, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
        match self.run_inner(o, out, err) {
            Ok(c) => c,
            Err(e) => {
                let _ = writeln!(err, "{}", e.0);
                1
            }
        }
    }

    fn run_inner(&mut self, o: &QueryOptions, out: &mut dyn Write, err: &mut dyn Write) -> Result<i32, FstError> {
        let w = |e: std::io::Error| FstError(format!("io: {e}"));
        if o.start > o.end {
            writeln!(err, "Invalid time range: --start ({}) is greater than --end ({})", o.start, o.end).map_err(w)?;
            return Ok(1);
        }
        let mut scope = o.scope.clone();
        while scope.ends_with('.') {
            scope.pop();
        }
        if o.info {
            return self.info(o, out).map(|_| 0);
        }
        let m = match Matcher::new(&o.patterns, o.regex, o.exact) {
            Ok(m) => m,
            Err(e) => {
                writeln!(err, "{}", e.0).map_err(w)?;
                return Ok(1);
            }
        };
        let scope_level = if scope.is_empty() { 0 } else { 1 + dots(&scope) };
        let depth = o.depth as usize;
        let in_scope = |path: &str, level: usize, is_signal: bool| -> bool {
            if !scope.is_empty() {
                let pre = format!("{scope}.");
                let under = path.starts_with(&pre);
                if is_signal && !under || !is_signal && path != scope && !under {
                    return false;
                }
            }
            if depth > 0 {
                let d = if scope.is_empty() {
                    level
                } else if is_signal {
                    if level >= scope_level { level - scope_level + 1 } else { 0 }
                } else {
                    level.saturating_sub(scope_level)
                };
                if d > depth {
                    return false;
                }
            }
            true
        };
        if o.list {
            if o.json {
                let mut items = Vec::new();
                if o.scopes_only {
                    for s in &self.scopes {
                        if in_scope(s, 1 + dots(s), false) && m.matches(s) {
                            items.push(json_str(s));
                        }
                    }
                } else {
                    for s in &self.signals {
                        if in_scope(&s.name, dots(&s.name), true) && m.matches(&s.name) {
                            items.push(format!(
                                "{{\"name\":{},\"width\":{},\"handle\":{},\"type\":{}}}",
                                json_str(&s.name),
                                s.width,
                                s.handle,
                                json_str(vt::name(s.kind))
                            ));
                        }
                    }
                }
                writeln!(out, "[{}]", items.join(",")).map_err(w)?;
                return Ok(0);
            }
            if o.scopes_only {
                for s in &self.scopes {
                    if in_scope(s, 1 + dots(s), false) && m.matches(s) {
                        writeln!(out, "{s}").map_err(w)?;
                    }
                }
            } else {
                for s in &self.signals {
                    if in_scope(&s.name, dots(&s.name), true) && m.matches(&s.name) {
                        writeln!(out, "{} [{}] (handle={})", s.name, s.width, s.handle).map_err(w)?;
                    }
                }
            }
            return Ok(0);
        }
        // Matched signals by handle, each handle's names in declaration order.
        let mut matched: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (i, s) in self.signals.iter().enumerate() {
            if in_scope(&s.name, dots(&s.name), true) && m.matches(&s.name) {
                matched.entry(s.handle).or_default().push(i);
            }
        }
        if matched.is_empty() {
            if o.count {
                if o.json {
                    writeln!(out, "{{\"transitions\":0}}").map_err(w)?;
                } else {
                    writeln!(out, "0 transitions").map_err(w)?;
                }
            }
            return Ok(0);
        }
        let print = |out: &mut dyn Write, t: u64, sample: bool, name: &str, v: &Value| -> std::io::Result<()> {
            if o.json {
                writeln!(
                    out,
                    "{{\"time\":{t},{}\"signal\":{},\"value\":{}}}",
                    if sample { "\"sample\":true," } else { "" },
                    json_str(name),
                    json_str(&value_text(v))
                )
            } else if sample {
                writeln!(out, "@{t} (sample) {name} = {}", value_text(v))
            } else {
                writeln!(out, "@{t} {name} = {}", value_text(v))
            }
        };
        let decodable = |s: &Signal| s.width > 0 || s.kind == vt::STRING || s.is_real();
        if !o.sample_at.is_empty() {
            let mut times = Vec::new();
            for tok in o.sample_at.split(|c: char| c == ',' || c.is_whitespace()) {
                if tok.is_empty() {
                    continue;
                }
                match parse_u64(tok) {
                    Some(v) => times.push(v),
                    None => {
                        writeln!(err, "Invalid --sample_at timestamp: '{tok}'").map_err(w)?;
                        return Ok(1);
                    }
                }
            }
            times.sort_unstable();
            times.dedup();
            let mut cur: BTreeMap<u32, Value> = BTreeMap::new();
            let mut snaps: Vec<BTreeMap<u32, Value>> = Vec::with_capacity(times.len());
            if let Some((t0, _)) = self.initial {
                while snaps.len() < times.len() && times[snaps.len()] < t0 {
                    snaps.push(cur.clone());
                }
                for &h in matched.keys() {
                    if let Some(v) = self.frame_value(h) {
                        cur.insert(h, v);
                    }
                }
            }
            let last = times.last().copied().unwrap_or(0);
            let want = |h: u32| matched.contains_key(&h);
            for b in self.reader.vc_blocks() {
                if !times.is_empty() && b.start > last {
                    break;
                }
                let d = self.reader.read_block(&b, &want, 0, last, false)?;
                for c in d.changes {
                    while snaps.len() < times.len() && times[snaps.len()] < c.time {
                        snaps.push(cur.clone());
                    }
                    cur.insert(c.handle, c.value);
                }
            }
            while snaps.len() < times.len() {
                snaps.push(cur.clone());
            }
            let x = Value::Bits(b"x".to_vec());
            for (k, &t) in times.iter().enumerate() {
                for (h, names) in &matched {
                    let v = snaps[k].get(h).unwrap_or(&x);
                    for &i in names {
                        print(out, t, true, &self.signals[i].name, v).map_err(w)?;
                    }
                }
            }
            return Ok(0);
        }
        // Transitions.
        let mut total = 0u64;
        let mut emitted = 0u64;
        let mut over = false;
        let mut tail: VecDeque<(u64, usize, Value)> = VecDeque::new();
        let mut first = true;
        let want = |h: u32| matched.contains_key(&h);
        'blocks: for b in self.reader.vc_blocks() {
            if b.end < o.start || b.start > o.end {
                first = false;
                continue;
            }
            let decoded = self.reader.read_block(&b, &want, o.start, o.end, false)?;
            let mut events: Vec<(u64, u32, Value)> = Vec::new();
            if first {
                if let Some((t0, _)) = self.initial {
                    if t0 >= o.start && t0 <= o.end {
                        let at_t0: HashSet<u32> =
                            decoded.changes.iter().filter(|c| c.time == t0).map(|c| c.handle).collect();
                        for (&h, names) in &matched {
                            if at_t0.contains(&h) || !decodable(&self.signals[names[0]]) {
                                continue;
                            }
                            if let Some(v) = self.frame_value(h) {
                                events.push((t0, h, v));
                            }
                        }
                    }
                }
            }
            first = false;
            events.extend(decoded.changes.into_iter().map(|c| (c.time, c.handle, c.value)));
            events.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
            for (t, h, v) in events {
                let Some(names) = matched.get(&h) else { continue };
                if o.count {
                    total += names.len() as u64;
                } else if o.tail > 0 {
                    for &i in names {
                        tail.push_back((t, i, v.clone()));
                        if tail.len() as u64 > o.tail {
                            tail.pop_front();
                        }
                    }
                } else {
                    for &i in names {
                        if o.limit > 0 && emitted >= o.limit {
                            over = true;
                            break 'blocks;
                        }
                        print(out, t, false, &self.signals[i].name, &v).map_err(w)?;
                        emitted += 1;
                    }
                }
            }
        }
        if o.count {
            if o.json {
                writeln!(out, "{{\"transitions\":{total}}}").map_err(w)?;
            } else {
                writeln!(out, "{total} transitions").map_err(w)?;
            }
            return Ok(0);
        }
        if o.tail > 0 {
            for (t, i, v) in &tail {
                print(out, *t, false, &self.signals[*i].name, v).map_err(w)?;
            }
            return Ok(0);
        }
        if over {
            writeln!(
                err,
                "[fst_query: Output limited to {} transitions. Use --limit 0 for unlimited, or narrow with --start/--end.]",
                o.limit
            )
            .map_err(w)?;
        }
        Ok(0)
    }

    fn info(&self, o: &QueryOptions, out: &mut dyn Write) -> Result<(), FstError> {
        let w = |e: std::io::Error| FstError(format!("io: {e}"));
        let h = &self.reader.header;
        if o.json {
            writeln!(
                out,
                "{{\"file\":{},\"writer\":{},\"date\":{},\"timescale\":{},\"timescale_text\":{},\"start\":{},\"end\":{},\"timezero\":{},\"scopes\":{},\"signals\":{},\"handles\":{},\"blocks\":{}}}",
                json_str(&self.path),
                json_str(&h.writer),
                json_str(&h.date),
                h.timescale,
                json_str(&format_timescale(h.timescale)),
                h.start_time,
                h.end_time,
                h.timezero,
                h.num_scopes,
                h.num_vars,
                h.max_handle,
                self.reader.vc_blocks().len()
            )
            .map_err(w)?;
            return Ok(());
        }
        let mut s = format!("File:         {}\n", self.path);
        if !h.writer.is_empty() {
            s.push_str(&format!("Writer:       {}\n", h.writer));
        }
        if !h.date.is_empty() {
            s.push_str(&format!("Date:         {}\n", h.date));
        }
        s.push_str(&format!("Timescale:    {}\n", format_timescale(h.timescale)));
        if h.start_time == u64::MAX {
            s.push_str("Time Range:   0 to 0\n");
        } else {
            s.push_str(&format!("Time Range:   {} to {}", h.start_time, h.end_time));
            let (base, mult) = split_timescale(h.timescale);
            let unit = time_unit(base);
            if !unit.is_empty() {
                if mult == 1 {
                    s.push_str(&format!(" {unit}"));
                } else {
                    s.push_str(&format!(
                        " ticks ({} to {} {unit})",
                        h.start_time.wrapping_mul(mult),
                        h.end_time.wrapping_mul(mult)
                    ));
                }
            }
            s.push('\n');
        }
        if h.timezero != 0 {
            s.push_str(&format!("Timezero:     {}\n", h.timezero));
        }
        s.push_str(&format!("Scopes:       {}\n", h.num_scopes));
        s.push_str(&format!("Signals:      {} (unique handles: {})\n", h.num_vars, h.max_handle));
        out.write_all(s.as_bytes()).map_err(w)
    }

    /// Run a batch of queries, one per line. Returns 0, or the last failing query's status.
    pub fn run_batch(&mut self, input: &mut dyn std::io::BufRead, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
        let mut status = 0;
        let mut line = String::new();
        loop {
            line.clear();
            match input.read_line(&mut line) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
            let t = line.trim_matches([' ', '\t', '\r', '\n']);
            if t.is_empty() || t.starts_with('#') {
                continue;
            }
            if t == "exit" || t == "quit" {
                break;
            }
            let toks = tokenize(t);
            let rc = match parse_args(&toks, false, err) {
                Ok(Parsed::Query(o, _)) => self.run(&o, out, err),
                Ok(Parsed::Help) => {
                    let _ = writeln!(out, "{}", HELP);
                    0
                }
                Err(ParseFailed) => 1,
            };
            if rc != 0 {
                status = rc;
            }
            let _ = writeln!(out, "--- [query end] ---");
            let _ = out.flush();
        }
        status
    }
}

/// The `fst` command's help, warning included.
pub const HELP: &str = "\
vyges loom fst — query an FST waveform in place

USAGE:
  vyges loom fst <file.fst> [flags...] [pattern...]
  vyges loom fst-write <in.vcd|in.fst> -o <out.fst> [--start T] [--end T] [--exact|--regex] [pattern...]
  vyges loom fst-to-vcd <in.fst> -o <out.vcd> --start T --end T <pattern...>

⛔ DO NOT CONVERT AN FST FILE TO VCD TO LOOK AT IT. A simulation's VCD is many times the size
   of its FST: it can fill the disk, and read into an AI agent's context it exhausts that too.
   Query the FST instead: --info reads only the header, --list the hierarchy, and a window
   query reads only the blocks it needs. fst-to-vcd exists for a tool that cannot read FST; it
   requires a time window AND a signal list, and stops at 256 MiB.

QUERIES:
  --info                 header only: writer, date, timescale, time range, counts
  --list                 signals as `name [width] (handle=h)`
    --scope S            only under scope S        --depth N   at most N levels below
    --scopes_only        scopes, not signals
  (default)              transitions `@<time> <name> = <value>`, by time then handle
    --start T --end T    inclusive window (default: the whole dump)
    --limit N            at most N lines (default 1000; 0 = unlimited); stderr says when it stopped
    --tail N             only the last N transitions in the window
    --count              print `N transitions` only
  --sample_at T1,T2,...  the value each signal holds at each time (`x` = none yet)
  --batch FILE|-         one query per line; each ends with `--- [query end] ---`
  --json                 machine-readable output

PATTERNS:
  substring of the full dotted name by default; --exact whole names; --regex a regular
  expression (anchored with --exact). Names carry no range: `top.core.pc`, not `pc [31:0]`.

VALUES:
  2..64 bits of 0/1 print as hex (0x1f); one bit or anything with x/z prints as levels; a real
  prints as %.16g; a string as its text.

WRITING:
  fst-write converts a VCD (streamed, never loaded whole) or copies a window and a signal
  subset of an FST into a new FST; each kept signal's value at the window start is its
  initial value.
";

/// `vyges loom fst …` — returns the exit status.
pub fn fst_main(args: &[String]) -> i32 {
    let stdout = std::io::stdout();
    let stderr = std::io::stderr();
    let mut out = std::io::BufWriter::new(stdout.lock());
    let mut err = stderr.lock();
    if args.is_empty() {
        let _ = writeln!(err, "Usage: vyges loom fst <fst_file> [flags...] [pattern...]  (try --help)");
        return 1;
    }
    let parsed = match parse_args(args, true, &mut err) {
        Ok(p) => p,
        Err(ParseFailed) => return 1,
    };
    let (mut o, batch) = match parsed {
        Parsed::Help => {
            let _ = write!(out, "{HELP}");
            return 0;
        }
        Parsed::Query(o, b) => (o, b),
    };
    if o.patterns.is_empty() {
        let _ = writeln!(err, "Usage: vyges loom fst <fst_file> [flags...] [pattern...]");
        return 1;
    }
    let path = o.patterns.remove(0);
    let mut s = match Session::open(&path) {
        Ok(s) => s,
        Err(e) => {
            let _ = writeln!(err, "Failed to open FST file: {path}: {}", e.0);
            return 1;
        }
    };
    let rc = match batch {
        Some(b) if b == "-" => {
            let stdin = std::io::stdin();
            let mut lock = stdin.lock();
            s.run_batch(&mut lock, &mut out, &mut err)
        }
        Some(b) => match std::fs::File::open(&b) {
            Ok(f) => {
                let mut r = std::io::BufReader::new(f);
                s.run_batch(&mut r, &mut out, &mut err)
            }
            Err(_) => {
                let _ = writeln!(err, "Failed to open batch query file: {b}");
                1
            }
        },
        None => s.run(&o, &mut out, &mut err),
    };
    let _ = out.flush();
    rc
}

/// Parse the write/export flags: `-o FILE`, `--start`, `--end`, `--exact`, `--regex`,
/// `--block-bytes`, patterns.
fn write_args(args: &[String]) -> Result<(String, String, crate::fst_writer::Selection, Option<usize>), String> {
    let mut sel = crate::fst_writer::Selection::default();
    let mut input = None;
    let mut output = None;
    let mut block = None;
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        i += 1;
        let val = |i: &mut usize| -> Result<String, String> {
            let v = args.get(*i).cloned().ok_or(format!("{a} needs a value"))?;
            *i += 1;
            Ok(v)
        };
        match a {
            "-o" | "--output" => output = Some(val(&mut i)?),
            "--start" => sel.start = parse_u64(&val(&mut i)?).ok_or("bad --start")?,
            "--end" => sel.end = Some(parse_u64(&val(&mut i)?).ok_or("bad --end")?),
            "--exact" => sel.exact = true,
            "--regex" => sel.regex = true,
            "--block-bytes" => block = Some(parse_u64(&val(&mut i)?).ok_or("bad --block-bytes")? as usize),
            s if s.starts_with('-') && s.len() > 1 => return Err(format!("unknown option {s}")),
            s => {
                if input.is_none() {
                    input = Some(s.to_string());
                } else {
                    sel.patterns.push(s.to_string());
                }
            }
        }
    }
    let input = input.ok_or("no input file")?;
    let output = output.ok_or("no output file (-o)")?;
    Ok((input, output, sel, block))
}

/// `vyges loom fst-write IN -o OUT [window] [patterns]`: VCD → FST, or an FST subset.
pub fn fst_write_main(args: &[String]) -> i32 {
    if args.iter().any(|a| a == "-h" || a == "--help") {
        print!("{HELP}");
        return 0;
    }
    let (input, output, sel, block) = match write_args(args) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges loom fst-write: {e} (try --help)");
            return 2;
        }
    };
    let file = match std::fs::File::create(&output) {
        Ok(f) => std::io::BufWriter::new(f),
        Err(e) => {
            eprintln!("vyges loom fst-write: {output}: {e}");
            return 1;
        }
    };
    let mut opts = crate::fst_writer::WriterOptions::default();
    if let Some(b) = block {
        opts.block_bytes = b;
    }
    let is_vcd = input.ends_with(".vcd") || input.ends_with(".vcd.gz");
    let res = if is_vcd {
        if !sel.patterns.is_empty() || sel.end.is_some() || sel.start != 0 {
            eprintln!("vyges loom fst-write: a VCD converts whole; write it, then copy a subset of the FST");
            return 2;
        }
        let f = match std::fs::File::open(&input) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("vyges loom fst-write: {input}: {e}");
                return 1;
            }
        };
        let mut r: Box<dyn std::io::BufRead> = if input.ends_with(".gz") {
            let mut all = Vec::new();
            if let Err(e) = std::io::Read::read_to_end(&mut std::io::BufReader::new(f), &mut all) {
                eprintln!("vyges loom fst-write: {input}: {e}");
                return 1;
            }
            match crate::fst_wave::inflate(&all) {
                Ok(t) => Box::new(std::io::Cursor::new(t)),
                Err(e) => {
                    eprintln!("vyges loom fst-write: {input}: {}", e.0);
                    return 1;
                }
            }
        } else {
            Box::new(std::io::BufReader::with_capacity(1 << 20, f))
        };
        crate::fst_writer::vcd_to_fst(&mut *r, opts, file).map(|(n, _)| n)
    } else {
        match FstReader::open(&input) {
            Ok(mut rd) => {
                opts.timescale = rd.header.timescale;
                opts.timezero = rd.header.timezero;
                opts.file_type = rd.header.file_type;
                match crate::fst_writer::FstWriter::new(file, opts) {
                    Ok(mut w) => crate::fst_writer::copy(&mut rd, &mut w, &sel).and_then(|n| w.finish().map(|_| n)),
                    Err(e) => Err(e),
                }
            }
            Err(e) => Err(e),
        }
    };
    match res {
        Ok(n) => {
            println!("wrote {output}: {n} signal(s)");
            0
        }
        Err(e) => {
            eprintln!("vyges loom fst-write: {}", e.0);
            1
        }
    }
}

/// `vyges loom fst-to-vcd IN -o OUT --start T --end T PATTERN...` — bounded, see [`HELP`].
pub fn fst_to_vcd_main(args: &[String]) -> i32 {
    if args.iter().any(|a| a == "-h" || a == "--help") {
        print!("{HELP}");
        return 0;
    }
    eprintln!(
        "⛔ fst-to-vcd: do not convert a dump to VCD to inspect it — query the FST (vyges loom fst). \
         This export is bounded to a window and a signal list."
    );
    let (input, output, sel, _) = match write_args(args) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges loom fst-to-vcd: {e} (try --help)");
            return 2;
        }
    };
    let mut r = match FstReader::open(&input) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("vyges loom fst-to-vcd: {}", e.0);
            return 1;
        }
    };
    if sel.end.is_none() || sel.patterns.is_empty() {
        eprintln!("vyges loom fst-to-vcd: needs --start/--end and at least one signal pattern");
        return 2;
    }
    let mut f = match std::fs::File::create(&output) {
        Ok(f) => std::io::BufWriter::new(f),
        Err(e) => {
            eprintln!("vyges loom fst-to-vcd: {output}: {e}");
            return 1;
        }
    };
    match crate::fst_writer::fst_to_vcd(&mut r, &sel, &mut f) {
        Ok((n, c)) => {
            let _ = f.flush();
            println!("wrote {output}: {n} signal(s), {c} change(s)");
            0
        }
        Err(e) => {
            eprintln!("vyges loom fst-to-vcd: {}", e.0);
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q(args: &[&str]) -> QueryOptions {
        let a: Vec<String> = args.iter().map(|s| s.to_string()).collect();
        match parse_args(&a, true, &mut Vec::new()) {
            Ok(Parsed::Query(o, _)) => o,
            _ => panic!("parse failed"),
        }
    }

    #[test]
    fn flags_parse_as_the_oracle_does() {
        let o = q(&["--start", "10", "-end=0x20", "--nolist", "--limit", "0", "a", "--", "--b"]);
        assert_eq!((o.start, o.end, o.limit), (10, 32, 0));
        assert_eq!(o.patterns, ["a", "--b"]);
        let mut e = Vec::new();
        assert!(parse_args(&["--start".into(), "-1".into()], true, &mut e).is_err());
        assert!(parse_args(&["--batch".into(), "x".into()], false, &mut e).is_err());
    }

    #[test]
    fn tokenizer_groups_quotes() {
        assert_eq!(tokenize("--regex 'a b' \"c\"d e"), ["--regex", "a b", "cd", "e"]);
    }

    #[test]
    fn timescale_text_matches() {
        assert_eq!(format_timescale(-12), "1 ps (1e-12 s)");
        assert_eq!(format_timescale(-10), "100 ps (1e-10 s)");
        assert_eq!(format_timescale(-8), "10 ns (1e-8 s)");
        assert_eq!(format_timescale(-18), "1e-18 s");
    }

    #[test]
    fn values_print_hex_only_for_2_to_64_binary_bits() {
        assert_eq!(value_text(&Value::Bits(b"1".to_vec())), "1");
        assert_eq!(value_text(&Value::Bits(b"0101".to_vec())), "0x5");
        assert_eq!(value_text(&Value::Bits(b"01x1".to_vec())), "01x1");
        assert_eq!(value_text(&Value::Bits(vec![b'1'; 65])), "1".repeat(65));
        assert_eq!(value_text(&Value::Real(1.0)), "1");
    }

    #[test]
    fn query_a_written_file() {
        use crate::fst_writer::{FstWriter, WriterOptions};
        let mut w = FstWriter::new(std::io::Cursor::new(Vec::new()), WriterOptions::default()).unwrap();
        w.scope(0, "TOP", "");
        let c = w.var(vt::WIRE, 0, "clk", 1, None);
        let d = w.var(vt::REG, 0, "d [7:0]", 8, None);
        w.upscope();
        w.time(0).unwrap();
        w.value(c, &Value::Bits(b"0".to_vec())).unwrap();
        w.value(d, &Value::Bits(b"00000000".to_vec())).unwrap();
        for t in 1..=10u64 {
            w.time(t * 10).unwrap();
            w.value(c, &Value::Bits(vec![b'0' + (t % 2) as u8])).unwrap();
            w.value(d, &Value::Bits(format!("{:08b}", t).into_bytes())).unwrap();
        }
        let bytes = w.finish().unwrap().into_inner();
        let p = std::env::temp_dir().join(format!("vyges-fstq-{}.fst", std::process::id()));
        std::fs::write(&p, bytes).unwrap();
        let mut s = Session::open(p.to_str().unwrap()).unwrap();
        let run = |s: &mut Session, args: &[&str]| -> (String, String) {
            let mut o = Vec::new();
            let mut e = Vec::new();
            s.run(&q(args), &mut o, &mut e);
            (String::from_utf8(o).unwrap(), String::from_utf8(e).unwrap())
        };
        let (o, _) = run(&mut s, &["--list"]);
        assert_eq!(o, "TOP.clk [1] (handle=1)\nTOP.d [8] (handle=2)\n");
        let (o, _) = run(&mut s, &["--start", "20", "--end", "30", "TOP.d"]);
        assert_eq!(o, "@20 TOP.d = 0x2\n@30 TOP.d = 0x3\n");
        let (o, e) = run(&mut s, &["--limit", "3"]);
        assert_eq!(o.lines().count(), 3);
        assert!(e.contains("Output limited to 3"));
        let (o, _) = run(&mut s, &["--count", "clk"]);
        assert_eq!(o, "11 transitions\n");
        let (o, _) = run(&mut s, &["--sample_at", "25,5", "--exact", "TOP.d"]);
        assert_eq!(o, "@5 (sample) TOP.d = 0x0\n@25 (sample) TOP.d = 0x2\n");
        let (o, _) = run(&mut s, &["--tail", "2", "TOP.d"]);
        assert_eq!(o, "@90 TOP.d = 0x9\n@100 TOP.d = 0xa\n");
        let _ = std::fs::remove_file(p);
    }
}
