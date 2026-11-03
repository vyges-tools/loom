//! The rest of the Tcl a vendor constraint file is written in: `return`, `upvar`, `dict`, procs
//! called for their value, `foreach_in_collection`, the tool-name test, and a signoff timer's
//! `remove_*` / `reset_*` spellings of the reference's `unset_*` / `delete_*` commands.
//!
//! Field item (second review, 2026-11): constraint files a DFT insertion tool generates were the
//! largest group the reader stopped on. They define their helpers as procs that `return` a value
//! or reach their caller's variables with `upvar`, keep their tables in `dict`s, walk collections
//! with `foreach_in_collection`, and branch on which tool is reading them. The reference reads SDC
//! in a full Tcl interpreter, where every one of these works; here each is read as Tcl runs it,
//! or — where only the design could decide — refused by name with the read going on.
//!
//! Each rule is a LOOM EXTENSION over the reference's own command set where the reference has no
//! such command (`foreach_in_collection`, `remove_*`, `get_app_var`); the Tcl built-ins
//! (`return`, `upvar`, `dict`) follow Tcl 8.6.

use std::collections::HashMap;

use super::{brace_word, control, dialect, logical_lines, subst_command_vars, subst_exprs, subst_vars, tokenize, Control, Dialect, ReadOpts, TclProc};

/// A command's words as Tcl substitutes them: a braced word is literal, and any other word has
/// its variables and `[expr …]` substituted INSIDE it — a variable whose value holds spaces (a
/// dict, a list) is still ONE word. (Substituting the line and splitting it after, a dict in a
/// variable became as many words as it has elements.)
fn tcl_words(line: &str, vars: &HashMap<String, String>) -> Vec<String> {
    let cs: Vec<char> = line.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < cs.len() {
        if cs[i].is_whitespace() {
            i += 1;
            continue;
        }
        let start = i;
        let braced = cs[i] == '{';
        let (mut brace, mut bracket, mut quote) = (0i32, 0i32, false);
        while i < cs.len() {
            match cs[i] {
                '\\' => i += 1,
                '{' if !quote => brace += 1,
                '}' if !quote => brace -= 1,
                '[' if brace == 0 => bracket += 1,
                ']' if brace == 0 => bracket -= 1,
                '"' if brace == 0 && bracket == 0 => quote = !quote,
                c if c.is_whitespace() && brace == 0 && bracket == 0 && !quote => break,
                _ => {}
            }
            i += 1;
        }
        let word: String = cs[start..i.min(cs.len())].iter().collect();
        if braced && word.ends_with('}') {
            out.push(word[1..word.len() - 1].to_string());
        } else {
            let w = word.strip_prefix('"').and_then(|w| w.strip_suffix('"')).unwrap_or(&word);
            out.push(subst_exprs(&subst_vars(w, vars), vars));
        }
    }
    out
}

/// The program a vendor file's tool-name test reads (`$synopsys_program_name`,
/// `[get_app_var synopsys_program_name]`) when the flow does not define it: a signoff timer, the
/// role every consumer of this reader plays. `--define synopsys_program_name=…` picks another.
pub(super) const TOOL_NAME_VAR: &str = "synopsys_program_name";
pub(super) const TOOL_NAME_DEFAULT: &str = "pt_shell";

/// A signoff timer's spelling of a take-back command, and the reference's command it is. The
/// arguments are the same in both (`-clock`, `-clock_fall`, `-rise`/`-fall`, `-min`/`-max`, the
/// objects); an option the reference's command lacks fails it there, and so it does here.
const TAKE_BACK_ALIASES: [(&str, &str); 14] = [
    ("remove_input_delay", "unset_input_delay"),
    ("remove_output_delay", "unset_output_delay"),
    ("remove_propagated_clock", "unset_propagated_clock"),
    ("remove_clock_uncertainty", "unset_clock_uncertainty"),
    ("remove_clock_latency", "unset_clock_latency"),
    ("remove_clock_transition", "unset_clock_transition"),
    ("remove_case_analysis", "unset_case_analysis"),
    ("remove_clock_groups", "unset_clock_groups"),
    ("remove_disable_timing", "unset_disable_timing"),
    ("remove_data_check", "unset_data_check"),
    ("remove_clock", "delete_clock"),
    ("remove_generated_clock", "delete_generated_clock"),
    ("reset_path", "unset_path_exceptions"),
    ("reset_timing_derate", "unset_timing_derate"),
];

/// The reference's command a signoff timer's `remove_*` / `reset_*` word is ([`TAKE_BACK_ALIASES`]).
pub(super) fn take_back_alias(word: &str) -> Option<&'static str> {
    TAKE_BACK_ALIASES.iter().find(|(from, _)| *from == word).map(|(_, to)| *to)
}

/// The line the reader expands into one take-back per clock defined so far (and none): what a
/// signoff timer's `remove_input_delay` / `remove_output_delay` WITHOUT `-clock` removes — every
/// input (output) delay on the objects — where the reference's `unset_*_delay` without `-clock`
/// removes only the delay that names no clock.
pub(super) const EVERY_CLOCK: &str = "\u{4}every-clock ";

/// `line` with its command word spelled as the reference's take-back command, when it is a
/// signoff timer's alias of one. `remove_input_delay` / `remove_output_delay` with no `-clock`
/// are marked [`EVERY_CLOCK`]: they take back the delays of every clock, on both edges.
pub(super) fn rewrite_take_back(line: &str) -> Option<String> {
    let t = line.trim_start();
    let head = t.split_whitespace().next()?;
    let to = take_back_alias(head)?;
    let every = matches!(head, "remove_input_delay" | "remove_output_delay") && !tokenize(t).iter().any(|w| w == "-clock");
    Some(format!("{}{to}{}", if every { EVERY_CLOCK } else { "" }, &t[head.len()..]))
}

/// An [`EVERY_CLOCK`] take-back as the reference's commands: the delay naming no clock, then each
/// clock's, rising and falling edge.
pub(super) fn every_clock_lines(line: &str, clocks: &[String]) -> Vec<String> {
    let t = line.trim_start();
    let head = t.split_whitespace().next().unwrap_or("");
    let rest = &t[head.len()..];
    let mut out = vec![t.to_string()];
    for c in clocks {
        let c = brace_word(c);
        out.push(format!("{head} -clock {c}{rest}"));
        out.push(format!("{head} -clock {c} -clock_fall{rest}"));
    }
    out
}

// ---- dict ----------------------------------------------------------------------------------

/// A dict value as Tcl reads it: a list of key/value pairs, in order, a later key replacing an
/// earlier one in place.
fn dict_pairs(v: &str) -> Result<Vec<(String, String)>, String> {
    let words = tokenize(v.trim());
    if !words.len().is_multiple_of(2) {
        return Err(format!("missing value to go with key (`{}` is not a dict)", v.trim()));
    }
    let mut out: Vec<(String, String)> = Vec::new();
    for kv in words.chunks(2) {
        match out.iter_mut().find(|(k, _)| *k == kv[0]) {
            Some(slot) => slot.1 = kv[1].clone(),
            None => out.push((kv[0].clone(), kv[1].clone())),
        }
    }
    Ok(out)
}

fn dict_text(pairs: &[(String, String)]) -> String {
    pairs.iter().map(|(k, v)| format!("{} {}", brace_word(k), brace_word(v))).collect::<Vec<_>>().join(" ")
}

/// `dict set` / `dict unset` on a key path: the dict with the innermost key replaced (or
/// removed), every dict on the way created as needed — Tcl's rule.
fn dict_put(d: &str, path: &[String], value: Option<&str>) -> Result<String, String> {
    let mut pairs = dict_pairs(d)?;
    let Some((key, rest)) = path.split_first() else { return Err("no key".into()) };
    let pos = pairs.iter().position(|(k, _)| k == key);
    if rest.is_empty() {
        match (value, pos) {
            (Some(v), Some(i)) => pairs[i].1 = v.to_string(),
            (Some(v), None) => pairs.push((key.clone(), v.to_string())),
            (None, Some(i)) => {
                pairs.remove(i);
            }
            (None, None) => {}
        }
    } else {
        let inner = pos.map(|i| pairs[i].1.clone()).unwrap_or_default();
        let new = dict_put(&inner, rest, value)?;
        match pos {
            Some(i) => pairs[i].1 = new,
            None => pairs.push((key.clone(), new)),
        }
    }
    Ok(dict_text(&pairs))
}

fn dict_get(d: &str, path: &[String]) -> Result<Option<String>, String> {
    let mut cur = d.to_string();
    for k in path {
        match dict_pairs(&cur)?.into_iter().find(|(kk, _)| kk == k) {
            Some((_, v)) => cur = v,
            None => return Ok(None),
        }
    }
    Ok(Some(cur))
}

/// `[dict SUB …]` for its value: `create`, `get`, `exists`, `keys`, `values`, `size`, `merge`,
/// `replace`. The words are already substituted.
fn dict_value(w: &[String]) -> Result<String, String> {
    let sub = w.first().map(String::as_str).unwrap_or("");
    match (sub, &w[1.min(w.len())..]) {
        ("create", kv) => {
            if kv.len() % 2 != 0 {
                return Err("dict create: wrong # args".into());
            }
            let pairs: Vec<(String, String)> = kv.chunks(2).map(|p| (p[0].clone(), p[1].clone())).collect();
            Ok(dict_text(&dict_pairs(&dict_text(&pairs))?))
        }
        ("get", [d, path @ ..]) => match dict_get(d, path)? {
            Some(v) => Ok(v),
            None => Err(format!("dict get: key \"{}\" not known in dictionary", path.join(" "))),
        },
        ("exists", [d, path @ ..]) if !path.is_empty() => Ok(if dict_get(d, path).ok().flatten().is_some() { "1" } else { "0" }.into()),
        ("keys", [d]) => Ok(dict_pairs(d)?.into_iter().map(|(k, _)| brace_word(&k)).collect::<Vec<_>>().join(" ")),
        ("values", [d]) => Ok(dict_pairs(d)?.into_iter().map(|(_, v)| brace_word(&v)).collect::<Vec<_>>().join(" ")),
        ("size", [d]) => Ok(dict_pairs(d)?.len().to_string()),
        ("merge", ds) => {
            let mut pairs: Vec<(String, String)> = Vec::new();
            for d in ds {
                for (k, v) in dict_pairs(d)? {
                    match pairs.iter_mut().find(|(kk, _)| *kk == k) {
                        Some(slot) => slot.1 = v,
                        None => pairs.push((k, v)),
                    }
                }
            }
            Ok(dict_text(&pairs))
        }
        ("replace", [d, kv @ ..]) if kv.len() % 2 == 0 => {
            let mut out = d.clone();
            for p in kv.chunks(2) {
                out = dict_put(&out, &p[..1], Some(&p[1]))?;
            }
            Ok(out)
        }
        _ => Err(format!("dict {sub}: not evaluated")),
    }
}

/// A `dict` command run as a statement: the commands it amounts to (`set` of the variable it
/// changes, or a `dict for` unrolled), or a refusal.
pub(super) fn dict_statement(raw: &str, vars: &HashMap<String, String>) -> Dialect {
    let toks = tcl_words(raw, vars);
    let sub = toks.get(1).map(String::as_str).unwrap_or("");
    let cur = |v: &String| vars.get(v.as_str()).cloned().unwrap_or_default();
    let set = |v: &String, val: String| Dialect::Run(vec![format!("set {v} {}", brace_word(&val))]);
    let res = match (sub, &toks[2.min(toks.len())..]) {
        ("set", [v, path @ .., value]) if !path.is_empty() => dict_put(&cur(v), path, Some(value)).map(|d| set(v, d)),
        ("unset", [v, path @ ..]) if !path.is_empty() => dict_put(&cur(v), path, None).map(|d| set(v, d)),
        ("lappend", [v, k, items @ ..]) => {
            let d = cur(v);
            let old = dict_get(&d, std::slice::from_ref(k)).ok().flatten().unwrap_or_default();
            let mut list: Vec<String> = tokenize(&old).iter().map(|e| brace_word(e)).collect();
            list.extend(items.iter().map(|e| brace_word(e)));
            dict_put(&d, std::slice::from_ref(k), Some(&list.join(" "))).map(|d| set(v, d))
        }
        ("append", [v, k, items @ ..]) => {
            let d = cur(v);
            let old = dict_get(&d, std::slice::from_ref(k)).ok().flatten().unwrap_or_default();
            dict_put(&d, std::slice::from_ref(k), Some(&format!("{old}{}", items.concat()))).map(|d| set(v, d))
        }
        ("incr", [v, k, by @ ..]) => {
            let d = cur(v);
            let old = dict_get(&d, std::slice::from_ref(k)).ok().flatten().unwrap_or_else(|| "0".into());
            match (old.trim().parse::<i64>(), by.first().map_or(Ok(1), |b| b.parse::<i64>())) {
                (Ok(a), Ok(b)) => dict_put(&d, std::slice::from_ref(k), Some(&(a + b).to_string())).map(|d| set(v, d)),
                _ => Err("dict incr: not an integer".into()),
            }
        }
        // `dict for {k v} $d {body}`: the body once per pair, in the dict's order
        ("for", [names, d, body]) => {
            let kv = tokenize(names);
            let [kn, vn] = kv.as_slice() else { return Dialect::Refuse("dict for: the variables are not `{key value}`; not run".into()) };
            dict_pairs(d).map(|pairs| {
                let body = logical_lines(body);
                let mut out = Vec::new();
                for (k, v) in pairs {
                    out.push(format!("set {kn} {}", brace_word(&k)));
                    out.push(format!("set {vn} {}", brace_word(&v)));
                    out.extend(body.iter().cloned());
                }
                Dialect::Run(out)
            })
        }
        // a value form run for nothing: Tcl evaluates it and drops the result
        (_, w) if dict_value(&std::iter::once(sub.to_string()).chain(w.iter().cloned()).collect::<Vec<_>>()).is_ok() => Ok(Dialect::Done),
        _ => Err(format!("dict {sub}: not evaluated")),
    };
    res.unwrap_or_else(|why| Dialect::Refuse(format!("{why}; not applied")))
}

// ---- substitutions for a value -------------------------------------------------------------

/// `line` with each `[…]` this reader can answer outside braces replaced by its value: `dict`
/// value forms, a proc the file defined (run for what it `return`s, [`proc_value`]) and the
/// tool-name query `[get_app_var synopsys_program_name]`. Any other substitution (`[get_pins …]`)
/// is left as text, as the reader always has. Inner substitutions are answered first, as Tcl
/// evaluates them. `Err` when a substitution here cannot be answered: the command is refused.
pub(super) fn subst_values(line: &str, vars: &HashMap<String, String>, procs: &HashMap<String, TclProc>, opts: &ReadOpts, depth: usize) -> Result<String, String> {
    if !line.contains('[') {
        return Ok(line.to_string());
    }
    let cs: Vec<char> = line.chars().collect();
    let mut out = String::with_capacity(line.len());
    let (mut i, mut brace) = (0usize, 0i32);
    while i < cs.len() {
        let c = cs[i];
        match c {
            '\\' => {
                out.extend(cs[i..(i + 2).min(cs.len())].iter());
                i += 2;
                continue;
            }
            '{' => brace += 1,
            '}' => brace = (brace - 1).max(0),
            '[' if brace == 0 => {
                // the matching `]`
                let (mut d, mut j) = (0i32, i);
                let mut b = 0i32;
                while j < cs.len() {
                    match cs[j] {
                        '\\' => j += 1,
                        '{' => b += 1,
                        '}' => b -= 1,
                        '[' if b == 0 => d += 1,
                        ']' if b == 0 => {
                            d -= 1;
                            if d == 0 {
                                break;
                            }
                        }
                        _ => {}
                    }
                    j += 1;
                }
                if j >= cs.len() {
                    out.extend(cs[i..].iter());
                    return Ok(out);
                }
                let inner: String = cs[i + 1..j].iter().collect();
                let head = inner.split_whitespace().next().unwrap_or("");
                let ours = head == "dict" || procs.contains_key(head) || (head == "get_app_var" && inner.contains(TOOL_NAME_VAR));
                if ours {
                    let inner = subst_values(&inner, vars, procs, opts, depth)?;
                    let words = tcl_words(&inner, vars);
                    let value = match head {
                        "dict" => dict_value(&words[1..])?,
                        "get_app_var" => vars.get(TOOL_NAME_VAR).cloned().unwrap_or_else(|| TOOL_NAME_DEFAULT.into()),
                        name => proc_value(name, &words[1..], vars, procs, opts, depth + 1)?,
                    };
                    out.push_str(&value_word(&value));
                } else {
                    // not ours (`[get_ports …]`): kept, with the substitutions inside it answered
                    out.push('[');
                    out.push_str(&subst_values(&inner, vars, procs, opts, depth)?);
                    out.push(']');
                }
                i = j + 1;
                continue;
            }
            _ => {}
        }
        out.push(c);
        i += 1;
    }
    Ok(out)
}

/// A substituted value as one word of the command it lands in. A COLLECTION is held as the query
/// text that makes it (`[get_pins a/CK]`, the reader's representation of an object set), so a
/// value that is one query stays a query, and several (`[get_pins a] [get_pins b]`, a list of
/// collections) are `[list …]` of them; any other value is one literal word, braced as needed.
fn value_word(v: &str) -> String {
    let t = v.trim();
    if t.starts_with('[') {
        // where the first `[` closes
        let mut d = 0i32;
        let mut close = None;
        for (k, c) in t.char_indices() {
            match c {
                '[' => d += 1,
                ']' => {
                    d -= 1;
                    if d == 0 {
                        close = Some(k);
                        break;
                    }
                }
                _ => {}
            }
        }
        if close == Some(t.len() - 1) {
            return t.to_string();
        }
        if t.ends_with(']') {
            return format!("[list {t}]");
        }
    }
    brace_word(v)
}

/// What a call of proc `name` with `args` RETURNS, run in its own frame: the parameters bound as
/// Tcl binds them, the body run until a `return` (its value) or its end (the last `set`, or the
/// last query, as Tcl returns its last command's result). Only commands that compute a value may
/// run here — `set`, `global`, `upvar`, `if`, `foreach`, `switch`, `dict`, `lappend`, `incr`,
/// `expr`, an object query — a proc called for its value that ALSO makes a constraint is refused,
/// since a substitution does not apply constraints in this reader.
pub(super) fn proc_value(name: &str, args: &[String], caller: &HashMap<String, String>, procs: &HashMap<String, TclProc>, opts: &ReadOpts, depth: usize) -> Result<String, String> {
    if depth > super::MAX_PROC_DEPTH {
        return Err(format!("{name}: called deeper than {}: a recursive proc", super::MAX_PROC_DEPTH));
    }
    let p = &procs[name];
    let mut vars = caller.clone();
    let mut a = args.iter();
    for (k, (pn, def)) in p.params.iter().enumerate() {
        let v = if pn == "args" && k + 1 == p.params.len() {
            Some(a.by_ref().map(|s| brace_word(s)).collect::<Vec<_>>().join(" "))
        } else {
            a.next().cloned().or_else(|| def.clone())
        };
        let Some(v) = v else { return Err(format!("{name}: no value for parameter `{pn}`")) };
        vars.insert(pn.clone(), v);
    }
    if a.next().is_some() {
        return Err(format!("{name}: more arguments than parameters"));
    }
    let mut queue: std::collections::VecDeque<String> = p.body.iter().cloned().collect();
    let mut last = String::new();
    let mut procs_local = procs.clone();
    let (mut frames, mut loops) = (Vec::new(), Vec::new());
    let mut steps = 0usize;
    while let Some(raw) = queue.pop_front() {
        steps += 1;
        if steps > super::MAX_LOOP_PASSES {
            return Err(format!("{name}: more than {} steps", super::MAX_LOOP_PASSES));
        }
        let raw = subst_values(&raw, &vars, &procs_local, opts, depth)?;
        let head = raw.split_whitespace().next().unwrap_or("").to_string();
        match head.as_str() {
            "return" => {
                let toks = tcl_words(&raw, &vars);
                // `return ?-code c? ?value?`: only a normal return carries a value back
                let value = toks.iter().skip(1).rfind(|t| !t.starts_with('-')).cloned().unwrap_or_default();
                return Ok(value);
            }
            "set" => {
                let toks = tcl_words(&raw, &vars);
                match toks.as_slice() {
                    [_, v, val] => {
                        vars.insert(v.clone(), val.clone());
                        last = val.clone();
                    }
                    [_, v] => last = vars.get(v.as_str()).cloned().unwrap_or_default(),
                    _ => return Err(format!("{name}: `{}` is not a set", raw.trim())),
                }
                continue;
            }
            "global" => continue, // the caller's variables are already visible here
            "upvar" => {
                for (local, other) in upvar_links(&raw)? {
                    if let Some(v) = vars.get(&other).cloned() {
                        vars.insert(local, v);
                    }
                }
                continue;
            }
            "dict" => match dict_statement(&raw, &vars) {
                Dialect::Run(cmds) => {
                    for c in cmds.into_iter().rev() {
                        queue.push_front(c);
                    }
                    continue;
                }
                Dialect::Done => continue,
                Dialect::Refuse(why) => return Err(why),
                _ => {}
            },
            "for" | "while" => return Err(format!("{name}: a loop in a proc called for its value")),
            _ => {}
        }
        // an object query as a statement: its result is the collection, kept as the query text
        if super::KNOWN_QUERIES.contains(&head.as_str()) {
            last = format!("[{}]", subst_command_vars(raw.trim(), &vars));
            continue;
        }
        match dialect(&raw, &vars, &mut procs_local, &mut frames, &mut loops, opts).map_err(|e| e.0)? {
            Dialect::Run(cmds) => {
                if !frames.is_empty() {
                    // a nested statement call: run it for its value, which is then dropped
                    frames.clear();
                    let args = tcl_words(&raw, &vars).split_off(1);
                    last = proc_value(&head, &args, &vars, &procs_local, opts, depth + 1)?;
                    continue;
                }
                for c in cmds.into_iter().rev() {
                    queue.push_front(c);
                }
                continue;
            }
            Dialect::Done => continue,
            Dialect::Refuse(why) => return Err(why),
            Dialect::Sourced(_) => return Err(format!("{name}: sources a file in a proc called for its value")),
            Dialect::Return => return Ok(String::new()),
            Dialect::None => {}
        }
        match control(&raw, &vars, &procs_local) {
            Control::Run(cmds) => {
                for c in cmds.into_iter().rev() {
                    queue.push_front(c);
                }
                continue;
            }
            Control::Refuse(kw) => return Err(format!("{name}: its `{kw}` is not evaluated")),
            Control::None => {}
        }
        if matches!(head.as_str(), "puts" | "echo" | "expr") {
            continue;
        }
        return Err(format!("{name}: called for its value, it runs `{head}`, which a substitution does not apply here"));
    }
    Ok(last)
}

/// `upvar ?level? other local ?other local …?` as (local, other) pairs. A level (`1`, `#0`) is
/// accepted; the link is to the caller's variable at any level — this reader's procs see their
/// caller's variables, and a `#0` link is a global one in the files read.
pub(super) fn upvar_links(raw: &str) -> Result<Vec<(String, String)>, String> {
    let toks = tokenize(raw);
    let mut rest: &[String] = &toks[1.min(toks.len())..];
    if let Some(first) = rest.first() {
        if rest.len() % 2 == 1 && (first.starts_with('#') || first.parse::<u32>().is_ok()) {
            rest = &rest[1..];
        }
    }
    if rest.is_empty() || !rest.len().is_multiple_of(2) {
        return Err(format!("upvar: `{}` is not `upvar ?level? other local …`", raw.trim()));
    }
    Ok(rest.chunks(2).map(|p| (p[1].trim_start_matches("::").to_string(), p[0].trim_start_matches("::").to_string())).collect())
}

/// Whether a `foreach_in_collection VAR … {body}` body can run ONCE with VAR set to the whole
/// collection and mean what running it once per object means. True when every body command is a
/// constraint (no Tcl of its own: no `set`, `if`, loop, proc call) and VAR appears only as a whole
/// word `$VAR` — an object argument. SDC constraint commands apply to each object of their object
/// arguments alike, so one command on the collection is the commands on each object; a body that
/// names, counts or tests the object (`get_object_name $p`, `clk_$p`) is not, and is refused.
pub(super) fn collection_body_is_setwise(var: &str, body: &[String], procs: &HashMap<String, TclProc>) -> bool {
    const NOT_SETWISE: &[&str] = &[
        "set", "if", "foreach", "for", "while", "switch", "proc", "return", "upvar", "global", "dict",
        "lappend", "incr", "foreach_in_collection", "append_to_collection", "create_clock",
        "create_generated_clock",
    ];
    let whole = [format!("${var}"), format!("${{{var}}}")];
    !body.is_empty()
        && body.iter().all(|line| {
            let toks = tokenize(line);
            let Some(head) = toks.first() else { return true };
            if NOT_SETWISE.contains(&head.as_str()) || procs.contains_key(head) || !super::is_command(head) {
                return false;
            }
            // VAR only as a whole word, never inside one
            toks.iter().skip(1).all(|t| whole.contains(t) || (!t.contains(&whole[0]) && !t.contains(&whole[1])))
        })
}
