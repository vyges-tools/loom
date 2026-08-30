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
    /// `inout` ports, kept SEPARATE from `inputs` and `outputs`.
    ///
    /// ⛔ **They were dropped entirely until 2026-08-30**, and the cost was measured: both
    /// edge-sensor SoC blocks declare `inout VPWR;` and `inout VGND;`, so this reader reported 322
    /// ports where OpenROAD built 324 block terminals from the same file.
    ///
    /// 🔑 **Separate on purpose, and it must stay that way.** Upstream maps a bidirect port to its
    /// own `dbIoType::INOUT` (`dbReadVerilog.cc:689`), not to input — and on our side `sta-si`
    /// reads `inputs`/`outputs` as timing endpoints, which an `inout` supply port is not. Folding
    /// them in would silently change timing.
    pub inouts: Vec<String>,
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
    //
    // THE BACKSLASH IS NOT PART OF THE NAME EITHER. Per the LRM an escaped identifier denotes
    // the same identifier as the equivalent simple one — `\foo ` IS `foo` — so it is dropped
    // here, exactly as the terminating whitespace is. Keeping it is not cosmetic: every other
    // file in the flow spells the name without it. A SPEF says `u_adapter.req_addr_q[0]` and a
    // DEF says `u_adapter\.req_addr_q\[0\]`, both of which resolve to the same characters,
    // and a netlist net called `\u_adapter.req_addr_q[0]` matches neither. On a real block that
    // was 767 of 14238 nets, and 4527 coupling aggressor references that a timer looked up,
    // failed to find, and dropped without a word — silently removing crosstalk from the
    // analysis.
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

/// Recover declared ranges from names already expanded to bits (`a[7]`…`a[0]` -> `a` is 7:0).
fn note_ranges(
    names: &[String],
    decl_range: &mut std::collections::BTreeMap<String, (i64, i64)>,
) {
    for nm in names {
        let Some((base, rest)) = nm.split_once('[') else { continue };
        let Some(b) = rest.strip_suffix(']').and_then(|x| x.parse::<i64>().ok()) else { continue };
        let e = decl_range.entry(base.to_string()).or_insert((b, b));
        e.0 = e.0.max(b);
        e.1 = e.1.min(b);
    }
}

/// The bits a connection expression covers, most significant first.
///
/// A bare name expands only when it is declared here as a vector — a black-box port's width is
/// unknowable, but the width of what is connected to it is not. A part-select `x[7:4]` expands
/// to its range; a bit-select `x[3]` and a constant are already one bit and pass through.
fn expand_bits(
    net: &str,
    decl_range: &std::collections::BTreeMap<String, (i64, i64)>,
) -> Vec<String> {
    if let Some((base, rest)) = net.split_once('[') {
        if let Some(inner) = rest.strip_suffix(']') {
            if let Some((a, b)) = inner.split_once(':') {
                if let (Ok(a), Ok(b)) = (a.trim().parse::<i64>(), b.trim().parse::<i64>()) {
                    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
                    return (lo..=hi).rev().map(|x| format!("{base}[{x}]")).collect();
                }
            }
        }
        return vec![net.to_string()];
    }
    match decl_range.get(net).copied() {
        Some((m, l)) => {
            let (lo, hi) = if m <= l { (m, l) } else { (l, m) };
            if hi > lo {
                (lo..=hi).rev().map(|x| format!("{net}[{x}]")).collect()
            } else {
                vec![net.to_string()]
            }
        }
        None => vec![net.to_string()],
    }
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
    // names declared as a one-bit VECTOR (`wire [0:0] x`), and the single index they carry
    let mut one_bit_vec: std::collections::BTreeMap<String, String> =
        std::collections::BTreeMap::new();
    // every declared signal's range, so a connection naming a whole bus can be expanded the way
    // a synthesiser expands it
    let mut decl_range: std::collections::BTreeMap<String, (i64, i64)> =
        std::collections::BTreeMap::new();
    let mut i = from;

    // module NAME ( ... ) ;  — keep name, skip the header port list
    if i < n && t[i] == "module" {
        i += 1;
        if i < n {
            nl.module = t[i].clone();
            i += 1;
        }
        // THE HEADER MAY BE WHERE THE PORTS ARE DECLARED. Verilog-2001 ANSI style puts the
        // direction, type and range inline — `module m (input wire [7:0] ui_in, output ...)` —
        // and a module written that way has no separate `input`/`output` statements at all.
        // Skipping the header then yields a design with ZERO ports: every join that asks whether
        // a net is a port fails, and the port-alias canonicalisation below is inert. It is not a
        // corner case — it is how modern Verilog is written, and how every TinyTapeout project
        // is written.
        //
        // Both styles are read: an ANSI header declares its ports here, and a non-ANSI header
        // just lists names that the `input`/`output` statements below will declare properly.
        if i < n && t[i] == "(" {
            i += 1;
            let mut depth = 1;
            let (mut dir, mut range): (Option<&str>, Option<(i64, i64)>) = (None, None);
            while i < n && depth > 0 {
                match t[i].as_str() {
                    "(" => depth += 1,
                    ")" => {
                        depth -= 1;
                        if depth == 0 {
                            i += 1;
                            break;
                        }
                    }
                    // a direction keyword starts a new ANSI declaration; the range and direction
                    // then apply to every name until the next one
                    d @ ("input" | "output" | "inout") => {
                        dir = Some(d);
                        range = None;
                    }
                    "wire" | "reg" | "logic" | "signed" => {}
                    "," => {}
                    "[" => {
                        let spec = t.get(i + 1).cloned().unwrap_or_default();
                        range = spec.split_once(':').and_then(|(h, l)| {
                            Some((h.trim().parse::<i64>().ok()?, l.trim().parse::<i64>().ok()?))
                        });
                        while i < n && t[i] != "]" {
                            i += 1;
                        }
                    }
                    tok if is_ident(tok) && !is_keyword(tok) => {
                        if let Some(d) = dir {
                            let names: Vec<String> = match range {
                                Some((h, l)) => {
                                    let (hi, lo) = if h >= l { (h, l) } else { (l, h) };
                                    decl_range.insert(tok.to_string(), (hi, lo));
                                    (lo..=hi).rev().map(|b| format!("{tok}[{b}]")).collect()
                                }
                                None => vec![tok.to_string()],
                            };
                            match d {
                                "input" => nl.inputs.extend(names),
                                "output" => nl.outputs.extend(names),
                                // ⚠️ Recorded, not folded into inputs: see `Netlist::inouts`.
                                "inout" => nl.inouts.extend(names),
                                _ => {}
                            }
                        }
                    }
                    _ => {}
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
            // The names come back already expanded to bits, so the declared range is recovered
            // from them — `expand_bits` needs it when an instance below connects the whole bus.
            "input" => {
                i += 1;
                let names = read_names(&mut i);
                note_ranges(&names, &mut decl_range);
                nl.inputs.extend(names);
            }
            "output" => {
                i += 1;
                let names = read_names(&mut i);
                note_ranges(&names, &mut decl_range);
                nl.outputs.extend(names);
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
            // A DECLARATION CARRIES THE WIDTH, and a one-bit VECTOR is named without an index.
            // `wire [0:0] clk;` and `wire clk;` are the same net to every consumer — Yosys's own
            // JSON represents them identically, `bits: [n]`, with nothing to tell them apart —
            // but Verilog source may write the connection either way. Left as written, one front
            // end says `clk[0]` and the other says `clk`, and the two describe the same design
            // differently. The declaration is the only place the answer exists, so it is read
            // rather than skipped.
            // ⛔ `inout` used to sit in the group below and was read as a plain wire, so its
            // names never reached any port list. It carries a range like the others, so the range
            // handling is shared — only the destination differs.
            "inout" => {
                i += 1;
                let names = read_names(&mut i);
                note_ranges(&names, &mut decl_range);
                nl.inouts.extend(names);
            }
            "wire" | "reg" | "parameter" | "localparam" | "supply0" | "supply1" => {
                i += 1;
                // an optional `[ msb:lsb ]` range, then the names it applies to
                let mut single: Option<String> = None;
                let mut range: Option<(i64, i64)> = None;
                if i < n && t[i] == "[" {
                    let spec = t.get(i + 1).cloned().unwrap_or_default();
                    if let Some((a, b)) = spec.split_once(':') {
                        if a.trim() == b.trim() {
                            single = Some(a.trim().to_string());
                        }
                        range = match (a.trim().parse::<i64>(), b.trim().parse::<i64>()) {
                            (Ok(x), Ok(y)) => Some((x, y)),
                            _ => None,
                        };
                    }
                    while i < n && t[i] != "]" {
                        i += 1;
                    }
                    i += 1;
                }
                while i < n && t[i] != ";" {
                    if is_ident(&t[i]) && !is_keyword(&t[i]) {
                        if let Some(idx) = &single {
                            one_bit_vec.insert(t[i].clone(), idx.clone());
                        }
                        if let Some(r) = range {
                            decl_range.insert(t[i].clone(), r);
                        }
                    }
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
                                // ONE RULE FOR EVERY CONNECTION SHAPE: flatten what is
                                // connected to a list of BITS, most significant first, then name
                                // the pin's bits from it. Verilog concatenation is MSB-first and
                                // that is how a bussed pin is named in Liberty, so the same
                                // numbering serves a concatenation, a whole bus, a part-select
                                // and any mixture.
                                //
                                // Doing it per MEMBER instead — `{uio_out[4:0], uo_out[4:0]}`
                                // becoming P[1] and P[0] — is wrong the moment a member is wider
                                // than one bit, which is most of the time. And a whole bus
                                // (`.data(ui_in)`) has to expand at all: the port's width is
                                // unknowable when the module is a black box, but the
                                // CONNECTION's is not, because the signal is declared here. A
                                // synthesiser infers the port from the expression the same way,
                                // which is why Yosys's JSON lists `data[0]`…`data[7]` where this
                                // reader listed one.
                                let bits: Vec<String> = nets
                                    .iter()
                                    .flat_map(|net| expand_bits(net, &decl_range))
                                    .collect();
                                match bits.len() {
                                    0 => {}
                                    1 => {
                                        if !is_const(&bits[0]) {
                                            conns.push((pin, bits[0].clone()));
                                        }
                                    }
                                    k => {
                                        for (idx, net) in bits.iter().enumerate() {
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
    // ---- canonicalise the net names, so both front ends describe one design ------------------
    //
    // Two normalisations, each because the same net is legitimately written more than one way
    // and every consumer joins by NAME:
    //
    //   * `x[0]` where `x` is a one-bit vector is just `x`. Yosys's JSON cannot say otherwise —
    //     it represents `wire [0:0] x` and `wire x` identically — so the bare form is the only
    //     one both front ends can agree on.
    //
    //   * a net tied to a PORT by `assign port = net;` is that port. Both names are the net's,
    //     and the port's is the one the DEF, the SPEF and the SDC all use, so it is the one that
    //     joins. Yosys resolves this for us by merging both names onto one bit id and reporting
    //     the port; reading the Verilog literally reported the local wire, and the two front
    //     ends then disagreed about a net on five of fifteen real designs.
    let ports: std::collections::BTreeSet<&str> =
        nl.inputs.iter().chain(nl.outputs.iter()).map(String::as_str).collect();
    let mut to_port: std::collections::BTreeMap<&str, &str> = std::collections::BTreeMap::new();
    for (lhs, rhs) in &nl.aliases {
        match (ports.contains(lhs.as_str()), ports.contains(rhs.as_str())) {
            (true, false) => to_port.insert(rhs.as_str(), lhs.as_str()),
            (false, true) => to_port.insert(lhs.as_str(), rhs.as_str()),
            // NEITHER IS A PORT — `assign rom0_data[2] = addr[3];` between two internal nets.
            // Still one net with two names, so the two front ends must still agree on which, and
            // there is no port to prefer. The driver is: `assign a = b` makes `b` the source and
            // `a` a name for it, which is also the one a synthesiser keeps when it merges them.
            (false, false) => to_port.insert(lhs.as_str(), rhs.as_str()),
            _ => None,
        };
    }
    let canon = |net: &str| -> String {
        if let Some(p) = to_port.get(net) {
            return p.to_string();
        }
        if let Some((base, rest)) = net.split_once('[') {
            if let Some(idx) = rest.strip_suffix(']') {
                if one_bit_vec.get(base).map(String::as_str) == Some(idx) {
                    return base.to_string();
                }
            }
        }
        net.to_string()
    };
    let renames: Vec<(usize, usize, String)> = nl
        .insts
        .iter()
        .enumerate()
        .flat_map(|(ii, inst)| {
            inst.conns.iter().enumerate().filter_map(move |(ci, (_, net))| {
                let c = canon(net);
                (c != *net).then_some((ii, ci, c))
            })
        })
        .collect();
    for (ii, ci, c) in renames {
        nl.insts[ii].conns[ci].1 = c;
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
        if depth_brace == 0 && closes > 0 {
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
        // Named WITHOUT the leading backslash: `\name ` and `name` are the same identifier in
        // Verilog, and every other file in the flow spells it the second way.
        assert_eq!(nl.insts[0].name, "clkbuf_0_gpio_in[0]");
        assert_eq!(nl.insts[1].cell, "sky130_fd_sc_hd__inv_2");
        assert_eq!(nl.insts[1].name, "ANTENNA_u_cpu.irq[1]");
        // no bogus numeric-cell instance leaked in
        assert!(nl.insts.iter().all(|inst| is_ident(&inst.cell)));
    }
}

#[cfg(test)]
mod inout_tests {
    use super::parse;

    /// ⛔ **`inout` ports were dropped entirely.** Measured on the edge-sensor SoC: both blocks
    /// declare `inout VPWR;` and `inout VGND;`, and this reader reported 322 ports where OpenROAD
    /// built 324 block terminals from the same netlist.
    #[test]
    fn standalone_inout_declarations_become_ports() {
        let nl = parse(
            "module m (a, y, VPWR, VGND);\n\
              input a;\n\
              output y;\n\
              inout VPWR;\n\
              inout VGND;\n\
              BUF u1 (.A(a), .X(y));\n\
             endmodule\n",
        )
        .unwrap();
        assert_eq!(nl.inputs, ["a"]);
        assert_eq!(nl.outputs, ["y"]);
        assert_eq!(nl.inouts, ["VPWR", "VGND"]);
    }

    /// 🔑 **They must NOT be folded into `inputs`.** Upstream gives a bidirect port its own
    /// `dbIoType::INOUT` (`dbReadVerilog.cc:689`), and `sta-si` reads `inputs`/`outputs` as timing
    /// endpoints — a supply port is not one. This test is what stops a later tidy-up merging them.
    #[test]
    fn inouts_stay_out_of_the_timing_endpoint_lists() {
        let nl = parse("module m (VPWR);\n inout VPWR;\nendmodule\n").unwrap();
        assert!(nl.inputs.is_empty(), "an inout is not an input");
        assert!(nl.outputs.is_empty(), "nor an output");
        assert_eq!(nl.inouts, ["VPWR"]);
    }

    /// The ANSI header form, where the direction is inline in the port list.
    #[test]
    fn ansi_header_inouts_are_read_too() {
        let nl = parse("module m (input a, output y, inout VPWR);\n BUF u1 (.A(a), .X(y));\nendmodule\n")
            .unwrap();
        assert_eq!(nl.inputs, ["a"]);
        assert_eq!(nl.outputs, ["y"]);
        assert_eq!(nl.inouts, ["VPWR"], "the ANSI path dropped these as well");
    }

    /// ⚠️ An `inout` carries a range like any other declaration, and the bits expand the same way.
    #[test]
    fn an_inout_bus_expands_to_bits() {
        let nl = parse("module m (io);\n inout [3:0] io;\nendmodule\n").unwrap();
        assert_eq!(nl.inouts, ["io[3]", "io[2]", "io[1]", "io[0]"]);
    }
}
