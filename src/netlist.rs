//! Structural (gate-level) Verilog reader — the connectivity STA needs.
//!
//! v0 reads a clean structural subset: the `module` header, `input`/`output`
//! port declarations, and cell instances of the form
//! `CELL inst ( .PIN(net), .PIN(net) );`. `wire`/`reg` declarations and the
//! header port list are skipped (port direction comes from `input`/`output`);
//! `assign`/`parameter` are skipped; a pin tied to a constant (`1'b0`) is recorded in
//! `Inst::ties`, not as a connection. Bus
//! ranges (`[7:0]`) are tolerated by skipping the range — v0 treats nets as
//! scalar. Pure std — fully unit-tested offline.

#[derive(Debug, Clone, Default)]
pub struct Inst {
    pub cell: String,
    pub name: String,
    pub conns: Vec<(String, String)>, // (pin, net)
    /// Pins tied to a CONSTANT, `(pin, value)` — `.E(1'b0)` gives `("E", false)`. Kept apart
    /// from `conns` because a constant is not a net; without them a tie cannot drive case
    /// analysis or constant propagation.
    ///
    /// 🔑 Bit for bit as the reference reader binds them to its `zero_`/`one_` nets: a constant
    /// is `size` bits wide (an unsized one is one bit), filled from its least significant digit
    /// with missing digits 0, and an `x`/`z` digit reads as 0. A bussed pin's bits are named
    /// `P[i]` exactly as `conns` names them. A positional constant has an empty pin name, as a
    /// positional net does.
    pub ties: Vec<(String, bool)>,
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
    /// Canonical names that arrived as ESCAPED identifiers (`\claimed[0] `).
    ///
    /// 🔑 **The name itself stays canonical** — see the tokenizer's note: `\foo ` IS `foo` per the
    /// LRM, and every other file in the flow spells it without the backslash. Keeping it once cost
    /// 767 of 14,238 nets and 4,527 coupling references that a timer looked up, failed to find and
    /// dropped silently. That behaviour must not change.
    ///
    /// ⚠️ **But a consumer writing OpenDB needs to know.** OpenROAD stores such a net as
    /// `claimed\[0\]` — each special character backslash-escaped — because the brackets are
    /// literal rather than a bus index. Without this set an importer cannot tell `claimed[0]` the
    /// escaped identifier from `claimed[0]` the bus bit, and the two are different nets.
    pub escaped_names: std::collections::BTreeSet<String>,
    pub insts: Vec<Inst>,
    /// Every module the file defines, in declaration order.
    ///
    /// Reading only the first was wrong on any hierarchical netlist: a file whose leaf cells
    /// come before the top gave a leaf as the design, silently.
    ///
    /// ⚠️ Filled on the TOP only (as is `escaped_names`): they describe the file, and a copy per
    /// module grows with the square of the module count. From [`parse_all`], read them through
    /// [`Design::module_names`] and [`Design::escaped_names`].
    pub modules: Vec<String>,
    /// `assign lhs = rhs ;` — two names for one net.
    ///
    /// Dropping these breaks connectivity rather than merely thinning it: a tool writes
    /// `assign out = n42;` and the output port has no driver at all as far as we are concerned.
    pub aliases: Vec<(String, String)>,
    /// **The renaming this reader applied**, as `written name -> canonical name`.
    ///
    /// 🔑 A net tied to a port by `assign port = net;` is reported under the PORT's name, because
    /// that is what a DEF, an SDC and *most* SPEFs call it, and because the two front ends
    /// otherwise disagreed about a net on five of fifteen real designs.
    ///
    /// ⛔ **But "most" is not "all", and the renaming is invisible to whoever has to join.**
    /// OpenROAD's SPEF for a routed sky130 block names those nets by the LOCAL wire — `net2007`,
    /// never `tl_o[2]` — so 53 nets' parasitics joined to nothing and were timed as ideal wire,
    /// with no symptom beyond an optimistic slack. Keeping the map is what lets a consumer try
    /// the other spelling instead of silently missing.
    pub canonical: std::collections::BTreeMap<String, String>,
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

/// The tokens of a netlist, ONE AT A TIME, each a slice of the text, with the values that
/// arrived as ESCAPED identifiers collected on the side.
/// ⚠️ The token VALUE is canonical either way; the set only records where a backslash was.
///
/// ⛔ **Tokens BORROW from the text, and are not all held at once.** An owned `String` per token
/// for the whole file, plus two cleaned copies of it, put the peak at many times the file size
/// before parsing began. Every delimiter is ASCII, so a token is a slice of the source; comments
/// are skipped in the same pass; and [`parse_all`] holds one module's tokens at a time.
struct Tokens<'a> {
    text: &'a str,
    it: std::str::CharIndices<'a>,
    /// a punctuation token found while closing the token before it
    queued: Option<&'a str>,
    /// inside an escaped identifier
    escaped: bool,
    escaped_vals: std::collections::BTreeSet<String>,
}

impl<'a> Tokens<'a> {
    fn new(text: &'a str) -> Self {
        Tokens { text, it: text.char_indices(), queued: None, escaped: false, escaped_vals: Default::default() }
    }

    /// Close the token open since `start`, if any, at byte `at`.
    fn cut(&mut self, start: &mut Option<usize>, at: usize) -> Option<&'a str> {
        let tok = start.take().filter(|&s0| s0 < at).map(|s0| &self.text[s0..at]);
        if let (Some(t), true) = (tok, self.escaped) {
            self.escaped_vals.insert(t.to_string());
        }
        self.escaped = false;
        tok
    }
}

impl<'a> Iterator for Tokens<'a> {
    type Item = &'a str;

    fn next(&mut self) -> Option<&'a str> {
        if let Some(q) = self.queued.take() {
            return Some(q);
        }
        let b = self.text.as_bytes();
        let mut start: Option<usize> = None;
        while let Some((i, ch)) = self.it.next() {
            // Comments — BOTH forms — end whatever token is open, escaped or not.
            //
            // `/* ... */` spans lines and is where tools park disabled code. Left in, its contents
            // tokenize like anything else, and `/* INV bogus (.A(a)); */` becomes a real instance in
            // the netlist: connectivity invented out of a comment, which no downstream tool can tell
            // from the real thing. An unterminated one runs to the end of the file.
            if ch == '/' && b.get(i + 1) == Some(&b'/') {
                let tok = self.cut(&mut start, i);
                for (_, c) in self.it.by_ref() {
                    if c == '\n' {
                        break;
                    }
                }
                if tok.is_some() {
                    return tok;
                }
                continue;
            }
            if ch == '/' && b.get(i + 1) == Some(&b'*') {
                let tok = self.cut(&mut start, i);
                self.it.next();
                let mut prev = '\0';
                for (_, c) in self.it.by_ref() {
                    if prev == '*' && c == '/' {
                        break;
                    }
                    prev = c;
                }
                if tok.is_some() {
                    return tok;
                }
                continue;
            }
            // A Verilog *escaped identifier* starts with `\` and runs until the next whitespace,
            // which terminates it (and is not part of the name). Punctuation inside it —
            // `\clkbuf_0_gpio_in[0]`, `\ANTENNA_u_cpu.foo[1]` — is part of the name, so it must NOT
            // be split on `[`/`]`/`.`; otherwise the real instance is missed and the leftover
            // `0 ] (` fragment mis-parses as a bogus cell.
            //
            // THE BACKSLASH IS NOT PART OF THE NAME EITHER. Per the LRM an escaped identifier
            // denotes the same identifier as the equivalent simple one — `\foo ` IS `foo` — so it is
            // dropped here, exactly as the terminating whitespace is. Keeping it is not cosmetic:
            // every other file in the flow spells the name without it. A SPEF says
            // `u_adapter.req_addr_q[0]` and a DEF says `u_adapter\.req_addr_q\[0\]`, both of which
            // resolve to the same characters, and a netlist net called `\u_adapter.req_addr_q[0]`
            // matches neither. On a real block that was 767 of 14238 nets, and 4527 coupling
            // aggressor references that a timer looked up, failed to find, and dropped without a
            // word — silently removing crosstalk from the analysis.
            if self.escaped {
                if ch.is_whitespace() {
                    if let Some(tok) = self.cut(&mut start, i) {
                        return Some(tok);
                    }
                } else if start.is_none() {
                    start = Some(i);
                }
                continue;
            }
            match ch {
                '\\' => {
                    let tok = self.cut(&mut start, i);
                    self.escaped = true;
                    if tok.is_some() {
                        return tok;
                    }
                }
                '(' | ')' | ';' | ',' | '.' | '[' | ']' | '=' => {
                    let punct = &self.text[i..i + 1];
                    return match self.cut(&mut start, i) {
                        Some(tok) => {
                            self.queued = Some(punct);
                            Some(tok)
                        }
                        None => Some(punct),
                    };
                }
                c if c.is_whitespace() => {
                    if let Some(tok) = self.cut(&mut start, i) {
                        return Some(tok);
                    }
                }
                _ => {
                    if start.is_none() {
                        start = Some(i);
                    }
                }
            }
        }
        // the last token — an escaped identifier at end of input has no whitespace to close it
        let end = self.text.len();
        self.cut(&mut start, end)
    }
}

/// A constant's bits, MOST significant first, as the reference reader parses it: `size` bits
/// (an unsized or unusable size is one bit), filled from the least significant digit with
/// missing digits 0. An `x`/`z` digit reads as 0, and so does every bit of a constant with an
/// unknown base or a decimal past 64 bits — each exactly as the reference does it.
fn const_bits(c: &str) -> Vec<bool> {
    // a size the reader would allocate without limit is not a netlist; cap it
    const MAX_BITS: usize = 1 << 16;
    let Some(q) = c.find('\'') else { return vec![false] };
    let size = c[..q]
        .trim_start_matches(['+', '-'])
        .replace('_', "")
        .parse::<usize>()
        .ok()
        .filter(|&n| n > 0)
        .unwrap_or(1)
        .min(MAX_BITS);
    let mut bits = vec![false; size]; // LSB first
    let rest = &c[q + 1..];
    let (base, digits) = match rest.chars().next() {
        Some(ch) => (ch.to_ascii_lowercase(), &rest[1..]),
        None => return bits,
    };
    let per_digit = match base {
        'b' => 1,
        'o' => 3,
        'h' => 4,
        'd' => {
            let d: String = digits.chars().filter(|&ch| ch != '_').collect();
            if let Ok(v) = d.parse::<u64>() {
                for (k, bit) in bits.iter_mut().enumerate().take(64) {
                    *bit = (v >> k) & 1 == 1;
                }
            }
            bits.reverse();
            return bits;
        }
        _ => {
            bits.reverse();
            return bits;
        }
    };
    let mut k = 0;
    for ch in digits.chars().rev().filter(|&ch| ch != '_') {
        let v = ch.to_digit(16).unwrap_or(0);
        for j in 0..per_digit {
            if k < size {
                bits[k] = (v >> j) & 1 == 1;
                k += 1;
            }
        }
        if k >= size {
            break;
        }
    }
    bits.reverse();
    bits
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

/// EVERY module a file defines, with the top marked.
///
/// ⛔ **`parse` returns only the top**, and on a hierarchical netlist that is not the design:
/// every non-top module's ports and instances are gone, so nothing below the top can be traced.
/// A consumer that walks the hierarchy reads this instead.
#[derive(Debug, Clone, Default)]
pub struct Design {
    /// Every module, in DECLARATION order. The file-wide lists (`modules`, `escaped_names`) are
    /// on the top only — see [`Design::module_names`].
    pub modules: Vec<Netlist>,
    /// Index into `modules` of the top: the module no other module instantiates.
    pub top: usize,
    /// `name -> index`, so a hierarchy walk looks each module up once rather than scanning.
    by_name: std::collections::HashMap<String, usize>,
}

impl Design {
    pub fn top(&self) -> &Netlist {
        &self.modules[self.top]
    }

    /// The module DEFINED under `name`, if the file defines it. An instance whose cell is not
    /// found here is a leaf — a library cell or a black box. A name defined twice gives the
    /// first definition.
    pub fn module(&self, name: &str) -> Option<&Netlist> {
        self.by_name.get(name).map(|&i| &self.modules[i])
    }

    /// Every module name, in declaration order (the top's `modules`).
    pub fn module_names(&self) -> &[String] {
        &self.top().modules
    }

    /// Names that arrived as escaped identifiers anywhere in the file (the top's `escaped_names`).
    pub fn escaped_names(&self) -> &std::collections::BTreeSet<String> {
        &self.top().escaped_names
    }
}

/// The top module only. See [`parse_all`] for a hierarchical netlist.
pub fn parse(text: &str) -> Result<Netlist, NetlistError> {
    let mut d = parse_all(text)?;
    Ok(d.modules.swap_remove(d.top))
}

/// Every module in the file, and which one is the top.
pub fn parse_all(text: &str) -> Result<Design, NetlistError> {
    let mut toks = Tokens::new(text);
    let mut mods: Vec<Netlist> = Vec::new();
    // ONE module's tokens at a time, `module` through `endmodule`; the buffer is reused, so the
    // peak is the largest module rather than the file.
    let mut t: Vec<&str> = Vec::new();
    // EVERY module, not just the first. A hierarchical netlist that lists its leaves before its
    // top otherwise hands back a leaf as the design.
    while let Some(m) = toks.by_ref().find(|&x| x == "module") {
        t.clear();
        t.push(m);
        for tok in toks.by_ref() {
            t.push(tok);
            if tok == "endmodule" {
                break;
            }
        }
        mods.push(parse_module(&t, 0).0);
    }
    let escaped_vals = std::mem::take(&mut toks.escaped_vals);
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
    let top = names
        .iter()
        .position(|nm| !instantiated.contains(nm.as_str()))
        // Every module instantiated by another (mutual recursion, or a single module that
        // instantiates itself) — fall back to the last, which is the usual emission order.
        .unwrap_or(mods.len() - 1);

    // ⛔ ONCE, on the top. Both describe the FILE, and a copy on every module is a list of n
    // names times n modules: tens of gigabytes on a netlist of tens of thousands of modules.
    let mut by_name = std::collections::HashMap::with_capacity(names.len());
    for (k, nm) in names.iter().enumerate() {
        by_name.entry(nm.clone()).or_insert(k);
    }
    mods[top].modules = names;
    mods[top].escaped_names = escaped_vals;
    Ok(Design { modules: mods, top, by_name })
}

/// Parse ONE `module … endmodule`, returning it and the index just past `endmodule`.
fn parse_module(t: &[&str], from: usize) -> (Netlist, usize) {
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
            nl.module = t[i].to_string();
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
                match t[i] {
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
                        let spec = t.get(i + 1).copied().unwrap_or_default();
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
            match t[*i] {
                "," => {}
                "[" => {
                    *i += 1;
                    let mut spec = String::new();
                    while *i < n && t[*i] != "]" {
                        spec.push_str(t[*i]);
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
        match t[i] {
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
                    let spec = t.get(i + 1).copied().unwrap_or_default();
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
                    if is_ident(t[i]) && !is_keyword(t[i]) {
                        if let Some(idx) = &single {
                            one_bit_vec.insert(t[i].to_string(), idx.clone());
                        }
                        if let Some(r) = range {
                            decl_range.insert(t[i].to_string(), r);
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
                        if k + 1 < n && is_ident(t[k]) && t[k + 1] == "(" {
                            head = k - 1; // so `head+1` is the instance name below
                        }
                    }
                }
                let cell_tok = i;
                if head + 2 < n
                    && is_ident(t[cell_tok])
                    && !is_keyword(t[cell_tok])
                    && !is_keyword(t[head + 1])
                    && t[head + 2] == "("
                {
                    let cell = t[cell_tok].to_string();
                    let name = t[head + 1].to_string();
                    i = head + 3; // past CELL [#(...)] INST (
                    let mut conns = Vec::new();
                    let mut ties = Vec::new();
                    let mut depth = 1;
                    // Named (`.PIN(net)`) and positional (`CELL u (a, b)`) forms are both legal
                    // and a netlist uses one or the other. Positional connections carry no pin
                    // name — that lives in the cell's LEF/Liberty — but the NET is what
                    // connectivity needs, so it is recorded with an empty pin and counted.
                    let named = (i..n).take_while(|&k| t[k] != ";").any(|k| t[k] == ".");
                    while i < n && depth > 0 {
                        match t[i] {
                            "(" => {
                                depth += 1;
                                i += 1;
                            }
                            ")" => {
                                depth -= 1;
                                i += 1;
                            }
                            "." if named => {
                                let pin = t.get(i + 1).copied().unwrap_or_default().to_string();
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
                                // A CONSTANT is as many bits as its size says, so a concatenation that
                                // holds one numbers its other members correctly; its bits are ties.
                                let bits: Vec<Result<String, bool>> = nets
                                    .iter()
                                    .flat_map(|net| -> Vec<Result<String, bool>> {
                                        if is_const(net) {
                                            const_bits(net).into_iter().map(Err).collect()
                                        } else {
                                            expand_bits(net, &decl_range).into_iter().map(Ok).collect()
                                        }
                                    })
                                    .collect();
                                let k = bits.len();
                                for (idx, bit) in bits.into_iter().enumerate() {
                                    let p = if k == 1 { pin.clone() } else { format!("{pin}[{}]", k - 1 - idx) };
                                    match bit {
                                        Ok(net) => conns.push((p, net)),
                                        Err(v) => ties.push((p, v)),
                                    }
                                }
                                i += 1;
                            }
                            _ if !named => {
                                let tok = t[i];
                                if is_const(tok) {
                                    ties.extend(const_bits(tok).into_iter().map(|v| (String::new(), v)));
                                    i += 1;
                                } else if is_ident(tok) {
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
                    nl.insts.push(Inst { cell, name, conns, ties });
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
    // ⚠️ Recorded even when nothing was renamed, so a consumer never has to ask whether the map
    // is populated — an empty map and "no renaming happened" are the same statement.
    nl.canonical =
        to_port.iter().map(|(from, to)| (from.to_string(), to.to_string())).collect();

    (nl, i)
}

/// One connection value: a net, a bit-select, or the members of a concatenation.
///
/// `i` enters at the first token of the value and leaves just past it. A bit-select arrives
/// from the tokenizer split into `count [ 7 ]` and is reassembled, so the name matches the
/// bus-expanded port and the bit-nets other gates drive.
fn read_conn_value(t: &[&str], i: &mut usize) -> Vec<String> {
    let n = t.len();
    let mut out = Vec::new();
    // A concatenation is `{ a , b[3] , c }`; the tokenizer keeps `{`/`}` inside a word, so it
    // arrives as `{a` … `c}` — trim the braces off the ends and take the members.
    let mut depth_brace = 0usize;
    loop {
        if *i >= n {
            break;
        }
        let tok = t[*i];
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
            if t.get(*i + 1).copied() == Some("[") {
                if let (Some(idx), Some("]")) =
                    (t.get(*i + 2), t.get(*i + 3).copied())
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
fn read_expr(t: &[&str], i: &mut usize) -> String {
    let mut j = *i;
    let v = read_conn_value(t, &mut j);
    *i = j;
    v.first().cloned().unwrap_or_default()
}

pub fn load(path: &str) -> Result<Netlist, NetlistError> {
    parse(&read_text(path)?)
}

/// [`load`], keeping every module — see [`Design`].
pub fn load_all(path: &str) -> Result<Design, NetlistError> {
    parse_all(&read_text(path)?)
}

/// A netlist file's text: gunzipped when it is gzip (with the `gzip` feature), and read
/// LOSSILY — a byte that is not UTF-8 becomes U+FFFD instead of failing the read.
///
/// ⛔ `read_to_string` refused both outright. Netlists are shipped gzipped, and a comment or a
/// string written in a legacy code page (Windows-1252) is enough to make a whole file "not
/// UTF-8" although every identifier in it is ASCII.
pub fn read_text(path: &str) -> Result<String, NetlistError> {
    let err = |e: String| NetlistError(format!("{path}: {e}"));
    let bytes = std::fs::read(path).map_err(|e| err(e.to_string()))?;
    let bytes = if bytes.starts_with(&[0x1f, 0x8b]) { gunzip(&bytes).map_err(err)? } else { bytes };
    Ok(match String::from_utf8(bytes) {
        Ok(s) => s,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    })
}

/// Every member of a gzip file, concatenated (RFC 1952; `bgzip` and `cat a.gz b.gz` write more
/// than one).
#[cfg(feature = "gzip")]
fn gunzip(mut data: &[u8]) -> Result<Vec<u8>, String> {
    use miniz_oxide::inflate::stream::{inflate, InflateState};
    use miniz_oxide::{DataFormat, MZFlush, MZStatus};
    let mut out = Vec::new();
    let mut buf = vec![0u8; 1 << 16];
    while data.starts_with(&[0x1f, 0x8b]) {
        let bad = || "malformed gzip header".to_string();
        let flg = *data.get(3).ok_or_else(bad)?;
        let mut p = 10usize;
        if flg & 0x04 != 0 {
            let x = data.get(p..p + 2).ok_or_else(bad)?;
            p += 2 + u16::from_le_bytes([x[0], x[1]]) as usize;
        }
        for flag in [0x08, 0x10] {
            if flg & flag != 0 {
                p += data.get(p..).ok_or_else(bad)?.iter().position(|&c| c == 0).ok_or_else(bad)? + 1;
            }
        }
        if flg & 0x02 != 0 {
            p += 2;
        }
        let mut input = data.get(p..).ok_or_else(bad)?;
        let mut st = InflateState::new_boxed(DataFormat::Raw);
        loop {
            let r = inflate(&mut st, input, &mut buf, MZFlush::None);
            input = &input[r.bytes_consumed..];
            out.extend_from_slice(&buf[..r.bytes_written]);
            match r.status {
                Ok(MZStatus::StreamEnd) => break,
                Ok(_) if r.bytes_consumed > 0 || r.bytes_written > 0 => {}
                Ok(_) => return Err("truncated gzip stream".into()),
                Err(e) => return Err(format!("gzip: {e:?}")),
            }
        }
        // the 8-byte trailer (CRC-32, size), then possibly another member
        data = input.get(8..).unwrap_or(&[]);
    }
    Ok(out)
}

#[cfg(not(feature = "gzip"))]
fn gunzip(_: &[u8]) -> Result<Vec<u8>, String> {
    Err("gzip-compressed; build vyges-loom with the `gzip` feature to read it".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule: a hierarchical netlist keeps EVERY module's contents; the top is the one nothing
    /// instantiates, wherever it is declared. (`parse` returned the top and dropped the rest, so
    /// nothing below it could be traced.)
    #[test]
    fn every_module_of_a_hierarchical_netlist_is_kept() {
        let src = r#"
module leaf (input a, output y);
  sky130_fd_sc_hd__inv_2 u0 (.A(a), .Y(y));
endmodule
module mid (input a, output y);
  wire n;
  leaf l0 (.a(a), .y(n));
  leaf l1 (.a(n), .y(y));
endmodule
module chip (input a, output y);
  mid m0 (.a(a), .y(y));
endmodule
"#;
        let d = parse_all(src).unwrap();
        assert_eq!(d.modules.len(), 3);
        assert_eq!(d.top().module, "chip");
        let mid = d.module("mid").expect("mid kept");
        assert_eq!(mid.inputs, vec!["a"]);
        assert_eq!(mid.insts.iter().map(|x| x.cell.as_str()).collect::<Vec<_>>(), ["leaf", "leaf"]);
        let leaf = d.module("leaf").expect("leaf kept");
        assert_eq!(leaf.insts[0].cell, "sky130_fd_sc_hd__inv_2");
        assert!(d.module("sky130_fd_sc_hd__inv_2").is_none(), "a library cell is a leaf, not a module");
        // `parse` still hands back the same top, unchanged.
        assert_eq!(parse(src).unwrap().module, "chip");
        // The file-wide lists are on the top ONCE, not copied onto every module.
        assert_eq!(d.module_names(), ["leaf", "mid", "chip"]);
        assert_eq!(parse(src).unwrap().modules, ["leaf", "mid", "chip"]);
        assert!(mid.modules.is_empty() && leaf.modules.is_empty());
    }

    /// Rule (the reference reader's): a constant is `size` bits, filled from its least
    /// significant digit, missing digits 0, `x`/`z` as 0, unsized = one bit; its bits are ties,
    /// and a concatenation counts a constant at its full width.
    #[test]
    fn constant_ties_are_recorded_bit_for_bit() {
        let src = r#"
module top (input [1:0] a, output y);
  DLY  u0 (.E(1'b0), .S(1'b1), .A(a[0]), .Y(y));
  MUX4 u1 (.D({a[1:0], 2'b10}), .S('b1), .T(4'hx), .U(3'd5));
  INV  u2 (1'b1, y);
endmodule
"#;
        let nl = parse(src).unwrap();
        let ties = |k: usize| nl.insts[k].ties.iter().map(|(p, v)| (p.as_str(), *v)).collect::<Vec<_>>();
        assert_eq!(ties(0), [("E", false), ("S", true)]);
        assert_eq!(nl.insts[0].conns, [("A".to_string(), "a[0]".to_string()), ("Y".into(), "y".into())]);
        assert_eq!(
            nl.insts[1].conns,
            [("D[3]".to_string(), "a[1]".to_string()), ("D[2]".into(), "a[0]".into())],
            "the constant's two bits are D[1:0], so the bus takes D[3:2]"
        );
        assert_eq!(
            ties(1),
            [("D[1]", true), ("D[0]", false), ("S", true), ("T[3]", false), ("T[2]", false), ("T[1]", false), ("T[0]", false),
             ("U[2]", true), ("U[1]", false), ("U[0]", true)]
        );
        assert_eq!(ties(2), [("", true)], "a positional constant, like a positional net, has no pin name");
        assert_eq!(nl.insts[2].conns, [(String::new(), "y".to_string())]);
    }

    #[test]
    fn const_bits_follow_the_reference() {
        assert_eq!(const_bits("1'b1"), [true]);
        assert_eq!(const_bits("4'b1_0"), [false, false, true, false], "missing digits are 0; `_` skipped");
        assert_eq!(const_bits("2'hF"), [true, true], "truncated to its size");
        assert_eq!(const_bits("'b1"), [true], "unsized is one bit");
        assert_eq!(const_bits("2'bxz"), [false, false]);
        assert_eq!(const_bits("2'sb11"), [false, false], "unknown base (here the sign) reads 0");
    }

    /// A non-UTF-8 byte (a Windows-1252 quote in a comment) no longer fails the whole read.
    #[test]
    fn a_file_that_is_not_utf8_is_read_lossily() {
        let dir = std::env::temp_dir().join(format!("loom-nl-lossy-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let f = dir.join("n.v");
        let mut bytes = b"// it\x92s a netlist\nmodule top (input a, output y);\n  INV u0 (.A(a), .Y(y));\nendmodule\n".to_vec();
        bytes[5] = 0x92;
        std::fs::write(&f, &bytes).unwrap();
        let d = load_all(f.to_str().unwrap()).unwrap();
        assert_eq!(d.top().insts.len(), 1);
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A gzipped netlist — two members, as `cat a.gz b.gz` writes — reads as its text.
    #[cfg(feature = "gzip")]
    #[test]
    fn a_gzipped_netlist_is_read() {
        fn member(text: &[u8]) -> Vec<u8> {
            let mut g = vec![0x1f, 0x8b, 8, 0x08, 0, 0, 0, 0, 0, 3];
            g.extend_from_slice(b"n.v\0"); // FNAME
            g.extend(miniz_oxide::deflate::compress_to_vec(text, 6));
            g.extend_from_slice(&[0; 4]); // CRC-32 (not checked)
            g.extend_from_slice(&(text.len() as u32).to_le_bytes());
            g
        }
        let mut gz = member(b"module leaf (input a, output y);\n  INV u0 (.A(a), .Y(y));\nendmodule\n");
        gz.extend(member(b"module top (input a, output y);\n  leaf l (.a(a), .y(y));\nendmodule\n"));
        let dir = std::env::temp_dir().join(format!("loom-nl-gz-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let f = dir.join("n.v.gz");
        std::fs::write(&f, gz).unwrap();
        let d = load_all(f.to_str().unwrap()).unwrap();
        assert_eq!(d.module_names(), ["leaf", "top"]);
        assert_eq!(d.top().module, "top");
        std::fs::remove_dir_all(&dir).ok();
    }

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

#[cfg(test)]
mod escaped_identifier_tests {
    use super::parse;

    /// ⛔ **The NAME stays canonical** — `\foo ` is `foo` per the LRM, and the tokenizer's own note
    /// records what keeping the backslash cost: 767 of 14,238 nets and 4,527 coupling references
    /// that resolved against nothing. This test is what stops that being undone.
    #[test]
    fn an_escaped_identifier_keeps_its_canonical_name() {
        let nl = parse("module m (a);\n input a;\n wire \\claimed[0] ;\n \
                        BUF u1 (.A(a), .X(\\claimed[0] ));\nendmodule\n")
            .unwrap();
        let nets: Vec<&str> =
            nl.insts[0].conns.iter().map(|(_, n)| n.as_str()).collect();
        assert!(nets.contains(&"claimed[0]"), "no backslash in the name: {nets:?}");
    }

    /// 🔑 **But WHICH names were escaped is recorded**, because a consumer writing OpenDB cannot
    /// otherwise tell the escaped identifier `claimed[0]` from the bus bit `claimed[0]` — and
    /// OpenROAD stores the first as `claimed\[0\]`.
    #[test]
    fn escaped_identifiers_are_reported_separately() {
        let nl = parse("module m (a);\n input a;\n wire \\claimed[0] ;\n wire [1:0] bus;\n \
                        BUF u1 (.A(a), .X(\\claimed[0] ));\nendmodule\n")
            .unwrap();
        assert!(nl.escaped_names.contains("claimed[0]"), "{:?}", nl.escaped_names);
        assert!(!nl.escaped_names.contains("bus[0]"), "a real bus bit is NOT escaped");
    }

    /// A dotted hierarchical name is the other common escaped form.
    #[test]
    fn a_dotted_escaped_name_is_recorded_too() {
        let nl = parse("module m (a);\n input a;\n \
                        BUF u1 (.A(a), .X(\\u_sub.n1 ));\nendmodule\n")
            .unwrap();
        assert!(nl.escaped_names.contains("u_sub.n1"), "{:?}", nl.escaped_names);
    }
}
