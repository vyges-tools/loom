//! Structural (gate-level) Verilog reader — the connectivity STA needs.
//!
//! v0 reads a clean structural subset: the `module` header, `input`/`output`
//! port declarations, and cell instances of the form
//! `CELL inst ( .PIN(net), .PIN(net) );`. `wire`/`reg` declarations and the
//! header port list are skipped (port direction comes from `input`/`output`);
//! `assign`/`parameter` are skipped; constant nets (`1'b0`) are dropped. Bus
//! ranges (`[7:0]`) are tolerated by skipping the range — v0 treats nets as
//! scalar. Pure std — fully unit-tested offline.

#[derive(Debug, Clone)]
pub struct Inst {
    pub cell: String,
    pub name: String,
    pub conns: Vec<(String, String)>, // (pin, net)
}

#[derive(Debug, Clone, Default)]
pub struct Netlist {
    /// The TOP module: the one no other module in the file instantiates.
    pub module: String,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    pub insts: Vec<Inst>,
    /// Every module the file defines, in declaration order.
    ///
    /// Reading only the first was wrong on any hierarchical netlist: a file whose leaf cells
    /// come before the top gave a leaf as the design, silently.
    pub modules: Vec<String>,
    /// `assign lhs = rhs ;` — two names for one net.
    ///
    /// Dropping these breaks connectivity rather than merely thinning it: a tool writes
    /// `assign out = n42;` and the output port has no driver at all as far as we are concerned.
    pub aliases: Vec<(String, String)>,
    /// Behavioural constructs seen and ignored (`always`, `initial`, `function`, `generate`).
    ///
    /// This reader is structural-only, and handed RTL it does not fail — it returns whatever
    /// fragments happen to look like instances. Measured on one real RTL file: 2 instances
    /// recovered from a module with 4 558 connections, silently. The count is how a caller
    /// finds out it passed the wrong kind of file.
    pub behavioural: usize,
    /// Connections given BY POSITION rather than by name, across all instances.
    ///
    /// They are recorded in `Inst::conns` with an EMPTY pin name, because the pin a position
    /// maps to is in the cell's LEF/Liberty, not in the netlist. The net is what connectivity
    /// needs; the count is here so a consumer that needs pin names knows they are missing.
    pub positional: usize,
}

impl Netlist {
    /// One line on anything a consumer should know before trusting this, or `None` when the
    /// read is clean.
    pub fn health(&self) -> Option<String> {
        let mut notes = Vec::new();
        if self.modules.len() > 1 {
            notes.push(format!(
                "{} modules in the file; `{}` taken as top (the one nothing instantiates)",
                self.modules.len(),
                self.module
            ));
        }
        if self.positional > 0 {
            notes.push(format!(
                "{} connection(s) given by position — the net is recorded, the pin name is not",
                self.positional
            ));
        }
        if !self.aliases.is_empty() {
            notes.push(format!(
                "{} `assign` alias(es) — resolve them or two names for one net stay separate",
                self.aliases.len()
            ));
        }
        if self.behavioural > 0 {
            notes.push(format!(
                "{} behavioural construct(s) ignored — this looks like RTL, not a gate-level \
                 netlist, and what was extracted from it is not a circuit",
                self.behavioural
            ));
        }
        (!notes.is_empty()).then(|| notes.join("; "))
    }
}

#[derive(Debug)]
pub struct NetlistError(pub String);
impl std::fmt::Display for NetlistError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "netlist error: {}", self.0)
    }
}
impl std::error::Error for NetlistError {}

/// Remove `/* ... */` comments, preserving line structure so line-based `//` stripping and any
/// future diagnostics still line up. Unterminated comments run to end of file, as Verilog says.
fn strip_block_comments(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let b = text.as_bytes();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'*' {
            i += 2;
            while i < b.len() && !(b[i] == b'*' && i + 1 < b.len() && b[i + 1] == b'/') {
                // keep newlines so line numbers and `//` handling are unaffected
                if b[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            i = (i + 2).min(b.len());
            continue;
        }
        out.push(b[i] as char);
        i += 1;
    }
    out
}

fn tokenize(text: &str) -> Vec<String> {
    // Strip comments first — BOTH forms.
    //
    // `/* ... */` spans lines and is where tools park disabled code. Left in, its contents
    // tokenize like anything else, and `/* INV bogus (.A(a)); */` becomes a real instance in
    // the netlist: connectivity invented out of a comment, which no downstream tool can tell
    // from the real thing.
    let no_block = strip_block_comments(text);
    let mut clean = String::with_capacity(no_block.len());
    for line in no_block.lines() {
        let l = match line.find("//") {
            Some(i) => &line[..i],
            None => line,
        };
        clean.push_str(l);
        clean.push('\n');
    }
    let mut out = Vec::new();
    let mut cur = String::new();
    let flush = |cur: &mut String, out: &mut Vec<String>| {
        if !cur.is_empty() {
            out.push(std::mem::take(cur));
        }
    };
    // A Verilog *escaped identifier* starts with `\` and runs until the next
    // whitespace, which terminates it (and is not part of the name). Punctuation
    // inside it — `\clkbuf_0_gpio_in[0]`, `\ANTENNA_u_cpu.foo[1]` — is part of the
    // name, so it must NOT be split on `[`/`]`/`.`; otherwise the real instance is
    // missed and the leftover `0 ] (` fragment mis-parses as a bogus cell.
    let mut escaped = false;
    for ch in clean.chars() {
        if escaped {
            if ch.is_whitespace() {
                flush(&mut cur, &mut out);
                escaped = false;
            } else {
                cur.push(ch);
            }
            continue;
        }
        match ch {
            '\\' => {
                flush(&mut cur, &mut out);
                cur.push(ch);
                escaped = true;
            }
            '(' | ')' | ';' | ',' | '.' | '[' | ']' | '=' => {
                flush(&mut cur, &mut out);
                out.push(ch.to_string());
            }
            c if c.is_whitespace() => flush(&mut cur, &mut out),
            c => cur.push(c),
        }
    }
    flush(&mut cur, &mut out);
    out
}

fn is_keyword(t: &str) -> bool {
    matches!(
        t,
        "module" | "endmodule" | "input" | "output" | "inout" | "wire" | "reg"
            | "assign" | "parameter" | "localparam" | "supply0" | "supply1"
    )
}

fn is_const(net: &str) -> bool {
    net.contains('\'') // 1'b0, 1'b1, etc.
}

/// A token that can begin a Verilog identifier (module / cell / instance / net
/// name): a letter, `_`, or the `\` of an escaped identifier. Numeric tokens
/// (bus indices like `0`, sized constants) are NOT identifiers — used so a stray
/// numeric token sitting before `(` is not mis-read as a cell instance.
fn is_ident(t: &str) -> bool {
    matches!(t.chars().next(), Some(c) if c.is_ascii_alphabetic() || c == '_' || c == '\\')
}

pub fn parse(text: &str) -> Result<Netlist, NetlistError> {
    let t = tokenize(text);
    let n = t.len();
    let mut mods: Vec<Netlist> = Vec::new();
    let mut i = 0;
    // EVERY module, not just the first. A hierarchical netlist that lists its leaves before its
    // top otherwise hands back a leaf as the design.
    while i < n {
        match t[i..].iter().position(|x| x == "module") {
            Some(p) => {
                let (m, next) = parse_module(&t, i + p);
                mods.push(m);
                i = next;
            }
            None => break,
        }
    }
    if mods.is_empty() {
        return Err(NetlistError("no module found".into()));
    }

    // TOP is the module no other module instantiates. That is the definition, and it does not
    // depend on declaration order — which is the only thing position could have told us.
    let names: Vec<String> = mods.iter().map(|m| m.module.clone()).collect();
    let instantiated: std::collections::BTreeSet<&str> = mods
        .iter()
        .flat_map(|m| m.insts.iter().map(|x| x.cell.as_str()))
        .collect();
    let top_idx = names
        .iter()
        .position(|nm| !instantiated.contains(nm.as_str()))
        // Every module instantiated by another (mutual recursion, or a single module that
        // instantiates itself) — fall back to the last, which is the usual emission order.
        .unwrap_or(mods.len() - 1);

    let mut top = mods.swap_remove(top_idx);
    top.modules = names;
    Ok(top)
}

/// Parse ONE `module … endmodule`, returning it and the index just past `endmodule`.
fn parse_module(t: &[String], from: usize) -> (Netlist, usize) {
    let n = t.len();
    let mut nl = Netlist::default();
    let mut i = from;

    // module NAME ( ... ) ;  — keep name, skip the header port list
    if i < n && t[i] == "module" {
        i += 1;
        if i < n {
            nl.module = t[i].clone();
            i += 1;
        }
        if i < n && t[i] == "(" {
            let mut depth = 0;
            while i < n {
                if t[i] == "(" {
                    depth += 1;
                } else if t[i] == ")" {
                    depth -= 1;
                    if depth == 0 {
                        i += 1;
                        break;
                    }
                }
                i += 1;
            }
        }
        if i < n && t[i] == ";" {
            i += 1;
        }
    }

    // Read a declaration's net names, **expanding bus ranges**: `output [7:0] count`
    // yields count[7]..count[0] as individual ports, so each bit matches the
    // bit-nets the gates drive (and SDC `set_output_delay [get_ports count[3]]`
    // resolves). A range stays in effect for every name in the same declaration
    // (`output [3:0] a, b;` -> a and b both expand). Scalars (no range) pass through.
    let read_names = |i: &mut usize| -> Vec<String> {
        let mut names = Vec::new();
        let mut range: Option<(i64, i64)> = None;
        while *i < n && t[*i] != ";" {
            match t[*i].as_str() {
                "," => {}
                "[" => {
                    *i += 1;
                    let mut spec = String::new();
                    while *i < n && t[*i] != "]" {
                        spec.push_str(&t[*i]);
                        *i += 1;
                    }
                    range = spec.split_once(':').and_then(|(h, l)| {
                        Some((h.trim().parse::<i64>().ok()?, l.trim().parse::<i64>().ok()?))
                    });
                }
                tok => match range {
                    Some((h, l)) => {
                        let (hi, lo) = if h >= l { (h, l) } else { (l, h) };
                        for b in (lo..=hi).rev() {
                            names.push(format!("{tok}[{b}]"));
                        }
                    }
                    None => names.push(tok.to_string()),
                },
            }
            *i += 1;
        }
        names
    };

    while i < n {
        match t[i].as_str() {
            "endmodule" => {
                i += 1;
                break;
            }
            "input" => {
                i += 1;
                nl.inputs.extend(read_names(&mut i));
            }
            "output" => {
                i += 1;
                nl.outputs.extend(read_names(&mut i));
            }
            // `assign lhs = rhs ;` is not decoration: it is two names for the same net, and
            // tools emit it for port aliases and constant ties. Skipping it leaves the lhs
            // with no driver.
            "assign" => {
                i += 1;
                let lhs = read_expr(t, &mut i);
                if i < n && t[i] == "=" {
                    i += 1;
                }
                let rhs = read_expr(t, &mut i);
                if !lhs.is_empty() && !rhs.is_empty() {
                    nl.aliases.push((lhs, rhs));
                }
                while i < n && t[i] != ";" {
                    i += 1;
                }
            }
            // Structural-only: these are not parsed, and their presence means the file is not
            // a gate-level netlist. Counted rather than ignored, so the caller is told.
            "always" | "always_ff" | "always_comb" | "always_latch" | "initial" | "function"
            | "task" | "generate" => {
                nl.behavioural += 1;
                i += 1;
            }
            "wire" | "reg" | "inout" | "parameter" | "localparam" | "supply0" | "supply1" => {
                while i < n && t[i] != ";" {
                    i += 1;
                }
            }
            ";" | ")" | "(" | "," | "." | "=" | "[" | "]" => i += 1,
            _ => {
                // candidate instance:  CELL  INST  ( .pin(net), ... ) ;
                // the cell must be a real identifier — a stray numeric token before
                // `(` is never a cell (defensive net around tokenizer edge cases).
                // A PARAMETER OVERRIDE sits between the cell and the instance name:
                // `ALU #(.MODE("AND")) u1 (.A(a), …)`. Yosys emits them, and so does any
                // parameterised macro. Read without allowing for it, `#` became the instance
                // NAME, the parameter list became its connections, and the instance's real
                // connections were never seen — a whole gate, silently replaced by its own
                // parameters.
                let mut head = i;
                if head + 1 < n && t[head + 1] == "#" {
                    let mut k = head + 2;
                    if k < n && t[k] == "(" {
                        let mut d = 0;
                        while k < n {
                            if t[k] == "(" {
                                d += 1;
                            } else if t[k] == ")" {
                                d -= 1;
                                if d == 0 {
                                    k += 1;
                                    break;
                                }
                            }
                            k += 1;
                        }
                        // `CELL #(…)` then INST `(` — keep the cell, skip the parameters
                        if k + 1 < n && is_ident(&t[k]) && t[k + 1] == "(" {
                            head = k - 1; // so `head+1` is the instance name below
                        }
                    }
                }
                let cell_tok = i;
                if head + 2 < n
                    && is_ident(&t[cell_tok])
                    && !is_keyword(&t[cell_tok])
                    && !is_keyword(&t[head + 1])
                    && t[head + 2] == "("
                {
                    let cell = t[cell_tok].clone();
                    let name = t[head + 1].clone();
                    i = head + 3; // past CELL [#(...)] INST (
                    let mut conns = Vec::new();
                    let mut depth = 1;
                    // Named (`.PIN(net)`) and positional (`CELL u (a, b)`) forms are both legal
                    // and a netlist uses one or the other. Positional connections carry no pin
                    // name — that lives in the cell's LEF/Liberty — but the NET is what
                    // connectivity needs, so it is recorded with an empty pin and counted.
                    let named = (i..n).take_while(|&k| t[k] != ";").any(|k| t[k] == ".");
                    while i < n && depth > 0 {
                        match t[i].as_str() {
                            "(" => {
                                depth += 1;
                                i += 1;
                            }
                            ")" => {
                                depth -= 1;
                                i += 1;
                            }
                            "." if named => {
                                let pin = t.get(i + 1).cloned().unwrap_or_default();
                                // expect '(' at i+2 — the value may be a single net, a
                                // bit-select, or a CONCATENATION on a bussed pin.
                                let mut j = i + 3;
                                let nets = read_conn_value(t, &mut j);
                                match nets.len() {
                                    0 => {}
                                    1 => {
                                        if !is_const(&nets[0]) {
                                            conns.push((pin, nets[0].clone()));
                                        }
                                    }
                                    // `{a, b, c}` on pin P is P[2], P[1], P[0] — Verilog
                                    // concatenation is MSB-first, and that is how a bussed pin
                                    // is named in Liberty. Keeping only the first member left a
                                    // net literally called `{a` and lost the rest.
                                    k => {
                                        for (idx, net) in nets.iter().enumerate() {
                                            if is_const(net) {
                                                continue;
                                            }
                                            conns.push((format!("{pin}[{}]", k - 1 - idx), net.clone()));
                                        }
                                    }
                                }
                                i += 1;
                            }
                            _ if !named => {
                                let tok = &t[i];
                                if is_ident(tok) && !is_const(tok) {
                                    let mut j = i;
                                    let nets = read_conn_value(t, &mut j);
                                    for net in nets {
                                        if !is_const(&net) {
                                            conns.push((String::new(), net));
                                            nl.positional += 1;
                                        }
                                    }
                                    i = j;
                                } else {
                                    i += 1;
                                }
                            }
                            _ => i += 1,
                        }
                    }
                    nl.insts.push(Inst { cell, name, conns });
                    if i < n && t[i] == ";" {
                        i += 1;
                    }
                } else {
                    i += 1;
                }
            }
        }
    }
    (nl, i)
}

/// One connection value: a net, a bit-select, or the members of a concatenation.
///
/// `i` enters at the first token of the value and leaves just past it. A bit-select arrives
/// from the tokenizer split into `count [ 7 ]` and is reassembled, so the name matches the
/// bus-expanded port and the bit-nets other gates drive.
fn read_conn_value(t: &[String], i: &mut usize) -> Vec<String> {
    let n = t.len();
    let mut out = Vec::new();
    // A concatenation is `{ a , b[3] , c }`; the tokenizer keeps `{`/`}` inside a word, so it
    // arrives as `{a` … `c}` — trim the braces off the ends and take the members.
    let mut depth_brace = 0usize;
    loop {
        if *i >= n {
            break;
        }
        let tok = t[*i].as_str();
        if tok == ")" || tok == ";" {
            break;
        }
        if tok == "," && depth_brace == 0 && !out.is_empty() {
            break;
        }
        if tok == "," {
            *i += 1;
            continue;
        }
        let opens = tok.matches('{').count();
        let closes = tok.matches('}').count();
        depth_brace += opens;
        let bare = tok.trim_matches(|c| c == '{' || c == '}');
        if !bare.is_empty() {
            let mut net = bare.to_string();
            // reassemble a bit-select that the tokenizer split
            if t.get(*i + 1).map(|s| s.as_str()) == Some("[") {
                if let (Some(idx), Some("]")) =
                    (t.get(*i + 2), t.get(*i + 3).map(|s| s.as_str()))
                {
                    net = format!("{net}[{idx}]");
                    *i += 3;
                }
            }
            out.push(net);
        }
        depth_brace = depth_brace.saturating_sub(closes);
        *i += 1;
        if depth_brace == 0 && (opens > 0 || closes > 0 || !out.is_empty()) && closes > 0 {
            break;
        }
        if depth_brace == 0 && opens == 0 && closes == 0 && out.len() == 1 {
            break;
        }
    }
    out
}

/// The left- or right-hand side of an `assign`, as a single name (bit-selects reassembled).
fn read_expr(t: &[String], i: &mut usize) -> String {
    let mut j = *i;
    let v = read_conn_value(t, &mut j);
    *i = j;
    v.first().cloned().unwrap_or_default()
}

pub fn load(path: &str) -> Result<Netlist, NetlistError> {
    let text = std::fs::read_to_string(path).map_err(|e| NetlistError(format!("{path}: {e}")))?;
    parse(&text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escaped_identifier_instance_names() {
        // Post-route netlists name buffered/antenna instances with Verilog escaped
        // identifiers that embed `[`, `]`, and `.` — `\clkbuf_0_gpio_in[0]`,
        // `\ANTENNA_u_cpu.irq[1]`. These must parse as ONE instance name; splitting
        // them shatters the real instance and mis-parses the `0 ] (` leftover as a
        // bogus cell named "0" (regression: "cell not in any .lib: 0").
        let src = r#"
module top (input a, output y);
  wire n1;
  sky130_fd_sc_hd__clkbuf_16 \clkbuf_0_gpio_in[0] (.A(a), .X(n1));
  sky130_fd_sc_hd__inv_2 \ANTENNA_u_cpu.irq[1] (.A(n1), .Y(y));
endmodule
"#;
        let nl = parse(src).unwrap();
        assert_eq!(nl.insts.len(), 2, "both escaped-id instances parse");
        assert_eq!(nl.insts[0].cell, "sky130_fd_sc_hd__clkbuf_16");
        assert_eq!(nl.insts[0].name, "\\clkbuf_0_gpio_in[0]");
        assert_eq!(nl.insts[1].cell, "sky130_fd_sc_hd__inv_2");
        assert_eq!(nl.insts[1].name, "\\ANTENNA_u_cpu.irq[1]");
        // no bogus numeric-cell instance leaked in
        assert!(nl.insts.iter().all(|inst| is_ident(&inst.cell)));
    }
}
