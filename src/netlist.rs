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
    /// `assign`s whose two sides are not the same width, and so were NOT merged — the reference
    /// warns and merges nothing (`mergeAssignNet`, warning 203), and so does this reader.
    pub skipped_assigns: usize,
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
        if self.skipped_assigns > 0 {
            notes.push(format!(
                "{} `assign`(s) with sides of different widths not merged, as the reference does",
                self.skipped_assigns
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

/// A netlist's tokens, read from a STREAM one at a time — the reference's design: its flex
/// scanner reads through `igzstream` and hands the parser one token at a time, so neither the
/// file nor its token list is ever held whole.
///
/// ⛔ **Nothing here holds the file.** A chip-scale netlist runs to many gigabytes; reading it
/// into a `String` first put the whole of it in memory before a token was produced, and an owned
/// token per word plus two cleaned copies put the peak at several times that. The lexer keeps
/// its state — comment, escape, a `/` waiting for the byte after it — across buffer refills, and
/// [`parse_reader`] writes one module's tokens into a reused arena and parses it there.
///
/// Bytes, not characters: every delimiter is ASCII, and whitespace is ASCII whitespace, as the
/// reference's lexer has it. A token that is not UTF-8 (a Windows-1252 byte) is read lossily.
struct Lexer<R: std::io::BufRead> {
    r: R,
    /// where the token being read starts in the arena — tokens are written there directly
    start: Option<usize>,
    /// the open token is an escaped identifier
    escaped: bool,
    /// a `/` seen, its meaning decided by the byte after it
    slash: bool,
    mode: Mode,
    /// a punctuation token found while closing the token before it
    pending: Option<u8>,
    /// only blanks (space, tab) since the last newline, or since the start of the file
    bol: bool,
    /// a `.` right after an identifier, its meaning decided by the byte after it
    dot: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Normal,
    LineComment,
    /// inside `/* */`; `true` when the last byte was `*`
    BlockComment(bool),
}

/// Bytes that end a run of an ordinary token: ASCII whitespace, `/` (a comment may start),
/// `\\` and the punctuation the parser reads as tokens of their own.
const STOP: [bool; 256] = stop_table(false);
/// The same inside an escaped identifier, which only whitespace (or a comment) ends.
const STOP_ESCAPED: [bool; 256] = stop_table(true);

const fn stop_table(escaped: bool) -> [bool; 256] {
    let mut t = [false; 256];
    let mut i = 0;
    while i < 256 {
        let b = i as u8;
        t[i] = b.is_ascii_whitespace()
            || b == b'/'
            || (!escaped && matches!(b, b'\\' | b'(' | b')' | b';' | b',' | b'.' | b'[' | b']' | b'='));
        i += 1;
    }
    t
}

/// A token's place in the arena, and whether it arrived as an ESCAPED identifier. ⚠️ The token
/// VALUE is canonical either way; the flag only records where a backslash was.
type Tok = (std::ops::Range<usize>, bool);

/// Close the token open since `start` in `arena`, or `None` when no token was open.
fn flush_tok(start: &mut Option<usize>, escaped: &mut bool, arena: &[u8]) -> Option<Tok> {
    let was_escaped = std::mem::replace(escaped, false);
    let s0 = start.take()?;
    Some((s0..arena.len(), was_escaped))
}

/// Append `b` to the open token in `arena`, opening one if none is.
fn push_tok(start: &mut Option<usize>, arena: &mut Vec<u8>, b: &[u8]) {
    if start.is_none() {
        *start = Some(arena.len());
    }
    arena.extend_from_slice(b);
}

impl<R: std::io::BufRead> Lexer<R> {
    fn new(r: R) -> Self {
        Lexer {
            r,
            start: None,
            escaped: false,
            slash: false,
            mode: Mode::Normal,
            pending: None,
            bol: true,
            dot: false,
        }
    }

    /// The next token, written to the end of `arena`; its range there, or `None` at the end.
    fn next_into(&mut self, arena: &mut Vec<u8>) -> std::io::Result<Option<Tok>> {
        loop {
            if let Some(p) = self.pending.take() {
                let s0 = arena.len();
                arena.push(p);
                return Ok(Some((s0..s0 + 1, false)));
            }
            let buf = self.r.fill_buf()?;
            if buf.is_empty() {
                if std::mem::take(&mut self.dot) {
                    self.pending = Some(b'.');
                    return Ok(flush_tok(&mut self.start, &mut self.escaped, arena));
                }
                // the end: a `/` still waiting is an ordinary byte, and an open token closes
                if std::mem::take(&mut self.slash) && self.mode == Mode::Normal {
                    push_tok(&mut self.start, arena, b"/");
                }
                return Ok(flush_tok(&mut self.start, &mut self.escaped, arena));
            }
            let (mut used, mut out) = (0usize, None);
            while used < buf.len() {
                // ⚡ whitespace between tokens is skipped in one go, and a run of ordinary token
                // bytes is copied in one go
                if self.mode == Mode::Normal && !self.slash && !self.dot && !self.escaped && self.start.is_none() {
                    let ws = buf[used..].iter().take_while(|b| b.is_ascii_whitespace()).count();
                    if ws > 0 {
                        let skipped = &buf[used..used + ws];
                        self.bol = match skipped.iter().rposition(|&b| b == b'\n') {
                            Some(k) => skipped[k + 1..].iter().all(|&b| b == b' ' || b == b'\t'),
                            None => self.bol && skipped.iter().all(|&b| b == b' ' || b == b'\t'),
                        };
                        used += ws;
                        continue;
                    }
                    // A line that starts with a compiler directive is dropped WHOLE, as the
                    // reference's lexer does (`^[ \t]*`.*{EOL}`): `` `timescale ``, `` `define ``,
                    // `` `ifdef `` and the rest are not evaluated, only removed — both arms of an
                    // `` `ifdef `` are read. Tokenised instead, `` `default_nettype wire `` made the
                    // next statement a `wire` declaration and the instance in it was lost.
                    if self.bol && buf[used] == b'`' {
                        self.mode = Mode::LineComment;
                        self.bol = false;
                        used += 1;
                        continue;
                    }
                }
                if self.mode == Mode::Normal && !self.slash && !self.dot {
                    // inside an escaped identifier only whitespace (and a comment) ends it
                    let stop = if self.escaped { STOP_ESCAPED } else { STOP };
                    let run = buf[used..].iter().take_while(|&&b| !stop[b as usize]).count();
                    if run > 0 {
                        self.bol = false;
                        push_tok(&mut self.start, arena, &buf[used..used + run]);
                        used += run;
                        continue;
                    }
                }
                let c = buf[used];
                used += 1;
                // Comments — BOTH forms — end whatever token is open, escaped or not.
                //
                // `/* ... */` spans lines and is where tools park disabled code. Left in, its contents
                // tokenize like anything else, and `/* INV bogus (.A(a)); */` becomes a real instance in
                // the netlist: connectivity invented out of a comment, which no downstream tool can tell
                // from the real thing. An unterminated one runs to the end of the file.
                match self.mode {
                    Mode::LineComment => {
                        if c == b'\n' {
                            self.mode = Mode::Normal;
                            self.bol = true;
                        }
                        continue;
                    }
                    Mode::BlockComment(star) => {
                        self.mode = if star && c == b'/' { Mode::Normal } else { Mode::BlockComment(c == b'*') };
                        continue;
                    }
                    Mode::Normal => {}
                }
                self.bol = c == b'\n' || (self.bol && (c == b' ' || c == b'\t'));
                if std::mem::take(&mut self.dot) {
                    if c.is_ascii_alphabetic() || c == b'_' {
                        push_tok(&mut self.start, arena, b".");
                    } else {
                        // not a hierarchical name: the identifier ends, the `.` is punctuation,
                        // and this byte is read again after it
                        self.pending = Some(b'.');
                        out = flush_tok(&mut self.start, &mut self.escaped, arena);
                        used -= 1;
                        break;
                    }
                }
                if std::mem::take(&mut self.slash) {
                    if c == b'/' || c == b'*' {
                        self.mode = if c == b'/' { Mode::LineComment } else { Mode::BlockComment(false) };
                        out = flush_tok(&mut self.start, &mut self.escaped, arena);
                        if out.is_some() {
                            break;
                        }
                        continue;
                    }
                    push_tok(&mut self.start, arena, b"/");
                }
                if c == b'/' {
                    self.slash = true;
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
                    if c.is_ascii_whitespace() {
                        out = flush_tok(&mut self.start, &mut self.escaped, arena);
                        if out.is_some() {
                            break;
                        }
                    } else {
                        push_tok(&mut self.start, arena, &[c]);
                    }
                    continue;
                }
                // A HIERARCHICAL name is one identifier: the reference lexer reads
                // `{ID_TOKEN}("."{ID_TOKEN})*` as a single ID, so `u1.n` is a net called `u1.n`.
                // Split, `.A(u1.n)` read as pin A on `u1` and a second pin `n`.
                if c == b'.' && !self.escaped && self.start.is_some_and(|s| arena[s].is_ascii_alphabetic() || arena[s] == b'_') {
                    self.dot = true;
                    continue;
                }
                match c {
                    b'\\' => {
                        out = flush_tok(&mut self.start, &mut self.escaped, arena);
                        self.escaped = true;
                    }
                    b'(' | b')' | b';' | b',' | b'.' | b'[' | b']' | b'=' => {
                        out = flush_tok(&mut self.start, &mut self.escaped, arena);
                        if out.is_some() {
                            self.pending = Some(c);
                        } else {
                            let s0 = arena.len();
                            arena.push(c);
                            out = Some((s0..s0 + 1, false));
                        }
                    }
                    c if c.is_ascii_whitespace() => {
                        out = flush_tok(&mut self.start, &mut self.escaped, arena)
                    }
                    _ => push_tok(&mut self.start, arena, &[c]),
                }
                if out.is_some() {
                    break;
                }
            }
            self.r.consume(used);
            if out.is_some() {
                return Ok(out);
            }
        }
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

/// The bits of `base[from:to]`, FROM first: the reference's `VerilogBusNetNameIterator` steps from
/// the left index to the right one, up or down, exactly as written. `[0:3]` is bits 0,1,2,3.
fn bus_bits(base: &str, from: i64, to: i64) -> Vec<String> {
    let step: i64 = if to >= from { 1 } else { -1 };
    let mut out = Vec::with_capacity((to - from).unsigned_abs() as usize + 1);
    let mut b = from;
    loop {
        out.push(format!("{base}[{b}]"));
        if b == to {
            break;
        }
        b += step;
    }
    out
}

/// The bits a connection expression covers, in the order the reference names them (left index first).
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
                    return bus_bits(base, a, b);
                }
            }
        }
        return vec![net.to_string()];
    }
    match decl_range.get(net).copied() {
        Some((m, l)) if m != l => bus_bits(net, m, l),
        Some(_) => vec![net.to_string()],
        None => vec![net.to_string()],
    }
}

fn is_keyword(t: &str) -> bool {
    matches!(
        t,
        "module" | "endmodule" | "input" | "output" | "inout" | "wire" | "reg"
            | "assign" | "parameter" | "localparam" | "supply0" | "supply1" | "tri" | "wand" | "wor"
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

/// A name in a [`StrPool`]: four bytes, compared and hashed as an integer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Sym(u32);

impl Sym {
    /// The symbol's index in its pool.
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Every distinct name of a design, stored ONCE: one text buffer, an end offset per name, and an
/// open-addressing index over them — about 16 bytes plus the text per distinct name.
///
/// ⛔ **Why.** The owned [`Netlist`] keeps a `String` for every cell name, instance name, pin and
/// net of every instance — about 460 bytes an instance, measured, nearly all of it repeats of the
/// same few cell and pin names and allocator overhead. A chip-scale netlist does not fit that way.
#[derive(Debug, Clone, Default)]
pub struct StrPool {
    text: String,
    ends: Vec<u64>,
    /// open addressing, linear probing; `EMPTY` is a free slot; at most half full
    table: Vec<u32>,
}

const EMPTY: u32 = u32::MAX;

/// FxHash over the bytes, eight at a time, with a final mix: fast, and good enough for an index.
fn fx_hash(s: &str) -> u64 {
    const K: u64 = 0x517c_c1b7_2722_0a95;
    let b = s.as_bytes();
    let mut h: u64 = b.len() as u64;
    let (words, rest) = b.as_chunks::<8>();
    for w in words {
        h = (h.rotate_left(5) ^ u64::from_le_bytes(*w)).wrapping_mul(K);
    }
    for &c in rest {
        h = (h.rotate_left(5) ^ c as u64).wrapping_mul(K);
    }
    // ⛔ Mixed before use: the index takes the LOW bits, and a multiply leaves them weak — short
    // names that differ in one byte (`w[3]`, `w[5]`) clustered into long probe chains.
    h ^= h >> 31;
    h = h.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    h ^ (h >> 29)
}

impl StrPool {
    /// The number of distinct names.
    pub fn len(&self) -> usize {
        self.ends.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ends.is_empty()
    }

    /// The text of `s`.
    pub fn get(&self, s: Sym) -> &str {
        let i = s.index();
        let a = if i == 0 { 0 } else { self.ends[i - 1] as usize };
        &self.text[a..self.ends[i] as usize]
    }

    /// The symbol for `s`, if the pool holds it.
    pub fn find(&self, s: &str) -> Option<Sym> {
        if self.table.is_empty() {
            return None;
        }
        let mask = self.table.len() - 1;
        let mut h = fx_hash(s) as usize & mask;
        loop {
            match self.table[h] {
                EMPTY => return None,
                e if self.get(Sym(e)) == s => return Some(Sym(e)),
                _ => h = (h + 1) & mask,
            }
        }
    }

    /// The symbol for `s`, adding it if new.
    pub fn intern(&mut self, s: &str) -> Sym {
        if self.ends.len() * 2 >= self.table.len() {
            self.grow();
        }
        let mask = self.table.len() - 1;
        let mut h = fx_hash(s) as usize & mask;
        loop {
            match self.table[h] {
                EMPTY => break,
                e if self.get(Sym(e)) == s => return Sym(e),
                _ => h = (h + 1) & mask,
            }
        }
        let id = u32::try_from(self.ends.len()).ok().filter(|&id| id != EMPTY).expect("more than 4 billion distinct names");
        self.text.push_str(s);
        self.ends.push(self.text.len() as u64);
        self.table[h] = id;
        Sym(id)
    }

    fn grow(&mut self) {
        let cap = (self.table.len() * 2).max(1024);
        self.table = vec![EMPTY; cap];
        let mask = cap - 1;
        for id in 0..self.ends.len() as u32 {
            let mut h = fx_hash(self.get(Sym(id))) as usize & mask;
            while self.table[h] != EMPTY {
                h = (h + 1) & mask;
            }
            self.table[h] = id;
        }
    }
}

/// One instance of a [`CompactModule`]: its cell and name, and where its connections and ties
/// lie in the module's tables. 20 bytes.
#[derive(Debug, Clone, Copy, Default)]
pub struct CompactInst {
    pub cell: Sym,
    pub name: Sym,
    conns: (u32, u32),
    ties: (u32, u32),
}

/// A module with every name a [`Sym`] and every instance's connections in ONE table — the
/// chip-scale form of [`Netlist`], with the same content. Read it through
/// [`CompactModule::conns`] / [`CompactModule::ties`] and the design's [`StrPool`].
#[derive(Debug, Clone, Default)]
pub struct CompactModule {
    pub name: Sym,
    pub inputs: Vec<Sym>,
    pub outputs: Vec<Sym>,
    pub inouts: Vec<Sym>,
    pub insts: Vec<CompactInst>,
    /// `(pin, net)` of every instance, in instance order — see [`Inst::conns`]
    conn_table: Vec<(Sym, Sym)>,
    /// `(pin, value)` of every instance — see [`Inst::ties`]
    tie_table: Vec<(Sym, bool)>,
    pub aliases: Vec<(Sym, Sym)>,
    /// see [`Netlist::skipped_assigns`]
    pub skipped_assigns: usize,
    /// written name -> canonical name — see [`Netlist::canonical`]
    pub canonical: Vec<(Sym, Sym)>,
    pub behavioural: usize,
    pub positional: usize,
}

impl CompactModule {
    /// `inst`'s `(pin, net)` connections — see [`Inst::conns`].
    pub fn conns(&self, inst: &CompactInst) -> &[(Sym, Sym)] {
        &self.conn_table[inst.conns.0 as usize..inst.conns.1 as usize]
    }

    /// `inst`'s constant ties — see [`Inst::ties`].
    pub fn ties(&self, inst: &CompactInst) -> &[(Sym, bool)] {
        &self.tie_table[inst.ties.0 as usize..inst.ties.1 as usize]
    }

    /// The owned form. `modules` / `escaped_names` are the DESIGN's, and are left empty here.
    pub fn to_netlist(&self, pool: &StrPool) -> Netlist {
        let st = |s: &Sym| pool.get(*s).to_string();
        Netlist {
            module: st(&self.name),
            inputs: self.inputs.iter().map(st).collect(),
            outputs: self.outputs.iter().map(st).collect(),
            inouts: self.inouts.iter().map(st).collect(),
            insts: self
                .insts
                .iter()
                .map(|x| Inst {
                    cell: st(&x.cell),
                    name: st(&x.name),
                    conns: self.conns(x).iter().map(|(p, n)| (st(p), st(n))).collect(),
                    ties: self.ties(x).iter().map(|(p, v)| (st(p), *v)).collect(),
                })
                .collect(),
            aliases: self.aliases.iter().map(|(a, b)| (st(a), st(b))).collect(),
            skipped_assigns: self.skipped_assigns,
            canonical: self.canonical.iter().map(|(a, b)| (st(a), st(b))).collect(),
            behavioural: self.behavioural,
            positional: self.positional,
            ..Default::default()
        }
    }
}

/// Every module of a file in the compact form — what [`Design`] is made from.
#[derive(Debug, Clone, Default)]
pub struct CompactDesign {
    /// every name the design uses, once
    pub pool: StrPool,
    /// in DECLARATION order
    pub modules: Vec<CompactModule>,
    /// index of the module no other module instantiates
    pub top: usize,
    /// one bit per name: arrived as an escaped identifier — see [`Netlist::escaped_names`]
    escaped: Vec<u64>,
    by_name: std::collections::HashMap<Sym, usize>,
}

impl CompactDesign {
    pub fn top(&self) -> &CompactModule {
        &self.modules[self.top]
    }

    /// The module DEFINED under `name`, if the file defines it (the first, if twice).
    pub fn module(&self, name: &str) -> Option<&CompactModule> {
        let s = self.pool.find(name)?;
        self.by_name.get(&s).map(|&i| &self.modules[i])
    }

    /// The text of a name.
    pub fn str(&self, s: Sym) -> &str {
        self.pool.get(s)
    }

    /// Did `s` arrive as an escaped identifier (`\foo[0] `)? See [`Netlist::escaped_names`].
    pub fn is_escaped(&self, s: Sym) -> bool {
        self.escaped.get(s.index() / 64).is_some_and(|w| w >> (s.index() % 64) & 1 == 1)
    }

    /// Every name that arrived as an escaped identifier, in pool order.
    pub fn escaped_names(&self) -> impl Iterator<Item = Sym> + '_ {
        (0..self.pool.len() as u32).map(Sym).filter(|s| self.is_escaped(*s))
    }

    /// The owned form of every module.
    pub fn to_design(&self) -> Design {
        let mut modules: Vec<Netlist> = self.modules.iter().map(|m| m.to_netlist(&self.pool)).collect();
        let names: Vec<String> = self.modules.iter().map(|m| self.pool.get(m.name).to_string()).collect();
        let mut by_name = std::collections::HashMap::with_capacity(names.len());
        for (k, nm) in names.iter().enumerate() {
            by_name.entry(nm.clone()).or_insert(k);
        }
        if let Some(t) = modules.get_mut(self.top) {
            t.modules = names;
            t.escaped_names = self.escaped_names().map(|s| self.pool.get(s).to_string()).collect();
        }
        Design { modules, top: self.top, by_name }
    }
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
    parse_reader(text.as_bytes())
}

/// [`parse_all`] from a STREAM — the file is never held whole; see [`Lexer`].
pub fn parse_reader<R: std::io::BufRead>(r: R) -> Result<Design, NetlistError> {
    Ok(parse_compact_reader(r)?.to_design())
}

/// Every module of a file in the COMPACT form — see [`CompactDesign`]. For a chip-scale netlist.
pub fn parse_compact(text: &str) -> Result<CompactDesign, NetlistError> {
    parse_compact_reader(text.as_bytes())
}

/// Record that a token arrived escaped: one bit per name in the pool, not a string per name —
/// on a netlist whose every name is escaped, a set of strings was most of the memory.
fn mark_escaped(pool: &mut StrPool, bits: &mut Vec<u64>, tok: &[u8]) {
    let s = pool.intern(&String::from_utf8_lossy(tok));
    let (w, b) = (s.index() / 64, s.index() % 64);
    if bits.len() <= w {
        bits.resize(w + 1, 0);
    }
    bits[w] |= 1 << b;
}

/// [`parse_compact`] from a STREAM: neither the file nor an owned copy of any module is held,
/// only the compact design.
pub fn parse_compact_reader<R: std::io::BufRead>(r: R) -> Result<CompactDesign, NetlistError> {
    let io = |e: std::io::Error| NetlistError(e.to_string());
    let mut lx = Lexer::new(r);
    let mut pool = StrPool::default();
    let mut escaped = Vec::new();
    let mut mods: Vec<CompactModule> = Vec::new();
    // ONE STATEMENT's tokens at a time — to its `;` at depth 0, or `endmodule` — in an arena
    // that is reused: the peak is the longest statement, not the module or the file.
    let mut arena: Vec<u8> = Vec::new();
    let mut spans: Vec<std::ops::Range<usize>> = Vec::new();
    // EVERY module, not just the first. A hierarchical netlist that lists its leaves before its
    // top otherwise hands back a leaf as the design.
    loop {
        arena.clear();
        spans.clear();
        let mut found = false;
        while let Some((t, esc)) = lx.next_into(&mut arena).map_err(io)? {
            if esc {
                mark_escaped(&mut pool, &mut escaped, &arena[t.clone()]);
            }
            if &arena[t.clone()] == b"module" {
                spans.push(t);
                found = true;
                break;
            }
            arena.clear();
        }
        if !found {
            break;
        }
        let mut reader = ModuleReader::default();
        let mut depth = 0i32;
        loop {
            let next = lx.next_into(&mut arena).map_err(io)?;
            let (done, end) = match next {
                None => (true, true),
                Some((t, esc)) => {
                    if esc {
                        mark_escaped(&mut pool, &mut escaped, &arena[t.clone()]);
                    }
                    let w = &arena[t.clone()];
                    let end = w == b"endmodule";
                    match w {
                        b"(" => depth += 1,
                        b")" => depth -= 1,
                        _ => {}
                    }
                    let done = end || (w == b";" && depth <= 0);
                    spans.push(t);
                    (done, end)
                }
            };
            if done && !spans.is_empty() {
                // UTF-8 is checked once per statement; one that is not (a Windows-1252 byte) is
                // read lossily, token by token.
                let lossy: Vec<String>;
                let toks: Vec<&str> = match std::str::from_utf8(&arena) {
                    Ok(text) => spans.iter().map(|t| &text[t.clone()]).collect(),
                    Err(_) => {
                        lossy = spans.iter().map(|t| String::from_utf8_lossy(&arena[t.clone()]).into_owned()).collect();
                        lossy.iter().map(String::as_str).collect()
                    }
                };
                reader.statement(&toks, &mut pool);
                arena.clear();
                spans.clear();
                depth = 0;
            }
            if end {
                break;
            }
        }
        mods.push(reader.finish(&mut pool));
    }
    if mods.is_empty() {
        return Err(NetlistError("no module found".into()));
    }

    // TOP is the module no other module instantiates. That is the definition, and it does not
    // depend on declaration order — which is the only thing position could have told us.
    let instantiated: std::collections::HashSet<Sym> = mods.iter().flat_map(|m| m.insts.iter().map(|x| x.cell)).collect();
    let top = mods
        .iter()
        .position(|m| !instantiated.contains(&m.name))
        // Every module instantiated by another (mutual recursion, or a single module that
        // instantiates itself) — fall back to the last, which is the usual emission order.
        .unwrap_or(mods.len() - 1);

    // ⛔ ONCE, on the top. Both describe the FILE, and a copy on every module is a list of n
    // names times n modules: tens of gigabytes on a netlist of tens of thousands of modules.
    // (the owned form keeps these on the top only — see `CompactDesign::to_design`)
    let mut by_name = std::collections::HashMap::with_capacity(mods.len());
    for (k, m) in mods.iter().enumerate() {
        by_name.entry(m.name).or_insert(k);
    }
    Ok(CompactDesign { pool, modules: mods, top, escaped, by_name })
}

/// One module, read ONE STATEMENT at a time — as the reference's parser consumes it: what
/// carries from one statement to the next is the module itself and its declarations (which
/// ranges, which one-bit vectors), never the tokens.
///
/// ⛔ Held whole, a flat module of a million and a half instances was some thirty million
/// tokens — a range and a slice each, about a gigabyte that existed only to be parsed.
#[derive(Default)]
struct ModuleReader {
    nl: CompactModule,
    // names declared as a one-bit VECTOR (`wire [0:0] x`), and the single index they carry
    one_bit_vec: std::collections::BTreeMap<String, String>,
    // every declared signal's range, so a connection naming a whole bus can be expanded the way
    // a synthesiser expands it
    decl_range: std::collections::BTreeMap<String, (i64, i64)>,
    /// names declared by a PORT declaration: a later `wire` range does not replace theirs
    /// (`VerilogModule::parseDcl` keeps the port's declaration over a wire "modifier")
    port_decl: std::collections::HashSet<String>,
    /// the header has been read: only the FIRST statement is `module NAME ( … ) ;`
    started: bool,
}

impl ModuleReader {
    /// One statement: the tokens up to and including its `;` (or `endmodule`); the first one is
    /// the header, `module NAME ( … ) ;`.
    fn statement(&mut self, t: &[&str], pool: &mut StrPool) {
        let n = t.len();
        let first = !std::mem::replace(&mut self.started, true);
        let nl = &mut self.nl;
        let one_bit_vec = &mut self.one_bit_vec;
        let decl_range = &mut self.decl_range;
        let port_decl = &mut self.port_decl;
        let mut i = 0;

        // module NAME ( ... ) ;  — keep name, skip the header port list
        if first && i < n && t[i] == "module" {
            i += 1;
            if i < n {
                nl.name = pool.intern(t[i]);
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
                                port_decl.insert(tok.to_string());
                                let names: Vec<String> = match range {
                                    Some((h, l)) => {
                                        decl_range.insert(tok.to_string(), (h, l));
                                        bus_bits(tok, h, l)
                                    }
                                    None => vec![tok.to_string()],
                                };
                                let names = names.iter().map(|x| pool.intern(x));
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
        // ... and each declared range, as written, for `expand_bits`
        let read_names = |i: &mut usize, decl_range: &mut std::collections::BTreeMap<String, (i64, i64)>, port_decl: &mut std::collections::HashSet<String>| -> Vec<String> {
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
                    tok => {
                        port_decl.insert(tok.to_string());
                        match range {
                            Some((h, l)) => {
                                decl_range.insert(tok.to_string(), (h, l));
                                names.extend(bus_bits(tok, h, l));
                            }
                            None => names.push(tok.to_string()),
                        }
                    }
                }
                *i += 1;
            }
            names
        };

        while i < n {
            match t[i] {
                "endmodule" => break,
                // The names come back already expanded to bits, so the declared range is recovered
                // from them — `expand_bits` needs it when an instance below connects the whole bus.
                "input" => {
                    i += 1;
                    let names = read_names(&mut i, decl_range, port_decl);
                    nl.inputs.extend(names.iter().map(|x| pool.intern(x)));
                }
                "output" => {
                    i += 1;
                    let names = read_names(&mut i, decl_range, port_decl);
                    nl.outputs.extend(names.iter().map(|x| pool.intern(x)));
                }
                // `assign lhs = rhs ;` is not decoration: it is two names for the same net, and
                // tools emit it for port aliases and constant ties. Skipping it leaves the lhs
                // with no driver.
                //
                // 🔑 BIT FOR BIT, as the reference's `mergeAssignNet` merges them: a whole bus, a
                // part-select and a concatenation on either side are expanded and paired, a
                // constant's bits tie, and a statement may hold several (`assign a = b, c = d;`).
                // Kept as one alias of the first names written, `assign w = x;` on buses joined no
                // bit, and a concatenation lost every member after its first.
                "assign" => {
                    i += 1;
                    loop {
                        let mut j = i;
                        let lhs = read_conn_value(t, &mut j);
                        i = j;
                        if i < n && t[i] == "=" {
                            i += 1;
                        }
                        let mut j = i;
                        let rhs = read_conn_value(t, &mut j);
                        i = j;
                        if !lhs.is_empty() && !rhs.is_empty() {
                            match assign_pairs(&lhs, &rhs, decl_range) {
                                Some(pairs) => nl.aliases.extend(pairs.iter().map(|(a, b)| (pool.intern(a), pool.intern(b)))),
                                None => nl.skipped_assigns += 1,
                            }
                        }
                        if i < n && t[i] == "," {
                            i += 1;
                            continue;
                        }
                        break;
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
                    let names = read_names(&mut i, decl_range, port_decl);
                    nl.inouts.extend(names.iter().map(|x| pool.intern(x)));
                }
                // `tri`, `wand` and `wor` declare nets as `wire` does (the reference grammar's
                // `dcl_type`); skipped, a `tri [3:0] t` connected whole was one bit
                kw @ ("wire" | "reg" | "parameter" | "localparam" | "supply0" | "supply1" | "tri" | "wand" | "wor") => {
                    let is_net = !matches!(kw, "reg" | "parameter" | "localparam");
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
                                if !port_decl.contains(t[i]) {
                                    decl_range.insert(t[i].to_string(), r);
                                }
                            }
                            // `wire a = b;` is an assignment too (the reference merges a
                            // declaration's assigns as `assign`'s); the right-hand side is read
                            // here so its names are not taken as declared
                            if t.get(i + 1) == Some(&"=") {
                                let lhs = vec![t[i].to_string()];
                                i += 2;
                                let rhs = read_conn_value(t, &mut i);
                                if is_net && !rhs.is_empty() {
                                    match assign_pairs(&lhs, &rhs, decl_range) {
                                        Some(pairs) => nl.aliases.extend(pairs.iter().map(|(a, b)| (pool.intern(a), pool.intern(b)))),
                                        None => nl.skipped_assigns += 1,
                                    }
                                }
                                continue;
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
                        let cell = pool.intern(t[cell_tok]);
                        let name = pool.intern(t[head + 1]);
                        i = head + 3; // past CELL [#(...)] INST (
                        // written straight into the module's tables
                        let (c0, t0) = (nl.conn_table.len() as u32, nl.tie_table.len() as u32);
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
                                                expand_bits(net, decl_range).into_iter().map(Ok).collect()
                                            }
                                        })
                                        .collect();
                                    let k = bits.len();
                                    for (idx, bit) in bits.into_iter().enumerate() {
                                        let p = if k == 1 { pin.clone() } else { format!("{pin}[{}]", k - 1 - idx) };
                                        let p = pool.intern(&p);
                                        match bit {
                                            Ok(net) => nl.conn_table.push((p, pool.intern(&net))),
                                            Err(v) => nl.tie_table.push((p, v)),
                                        }
                                    }
                                    i += 1;
                                }
                                _ if !named => {
                                    let tok = t[i];
                                    if is_const(tok) {
                                        let none = pool.intern("");
                                        nl.tie_table.extend(const_bits(tok).into_iter().map(|v| (none, v)));
                                        i += 1;
                                    } else if is_ident(tok) {
                                        let mut j = i;
                                        let nets = read_conn_value(t, &mut j);
                                        for net in nets {
                                            if !is_const(&net) {
                                                let none = pool.intern("");
                                                nl.conn_table.push((none, pool.intern(&net)));
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
                        let (c1, t1) = (nl.conn_table.len() as u32, nl.tie_table.len() as u32);
                        nl.insts.push(CompactInst { cell, name, conns: (c0, c1), ties: (t0, t1) });
                        if i < n && t[i] == ";" {
                            i += 1;
                        }
                    } else {
                        i += 1;
                    }
                }
            }
        }

    }

    /// The module, its net names canonicalised now that every `assign` has been read.
    fn finish(self, pool: &mut StrPool) -> CompactModule {
        let mut nl = self.nl;
        let one_bit_vec = self.one_bit_vec;
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
        let ports: std::collections::BTreeSet<Sym> = nl.inputs.iter().chain(nl.outputs.iter()).copied().collect();
        let mut to_port: std::collections::BTreeMap<Sym, Sym> = std::collections::BTreeMap::new();
        for &(lhs, rhs) in &nl.aliases {
            match (ports.contains(&lhs), ports.contains(&rhs)) {
                (true, false) => to_port.insert(rhs, lhs),
                (false, true) => to_port.insert(lhs, rhs),
                // NEITHER IS A PORT — `assign rom0_data[2] = addr[3];` between two internal nets.
                // Still one net with two names, so the two front ends must still agree on which, and
                // there is no port to prefer. The driver is: `assign a = b` makes `b` the source and
                // `a` a name for it, which is also the one a synthesiser keeps when it merges them.
                (false, false) => to_port.insert(lhs, rhs),
                _ => None,
            };
        }
        for k in 0..nl.conn_table.len() {
            let net = nl.conn_table[k].1;
            if let Some(&p) = to_port.get(&net) {
                nl.conn_table[k].1 = p;
                continue;
            }
            let text = pool.get(net);
            if let Some((base, rest)) = text.split_once('[') {
                if let Some(idx) = rest.strip_suffix(']') {
                    if one_bit_vec.get(base).map(String::as_str) == Some(idx) {
                        let base = base.to_string();
                        nl.conn_table[k].1 = pool.intern(&base);
                    }
                }
            }
        }
        // ⚠️ Recorded even when nothing was renamed, so a consumer never has to ask whether the map
        // is populated — an empty map and "no renaming happened" are the same statement. Ordered by
        // the WRITTEN name, as the owned map is.
        let mut canonical: Vec<(Sym, Sym)> = to_port.into_iter().collect();
        canonical.sort_by(|a, b| pool.get(a.0).cmp(pool.get(b.0)));
        nl.canonical = canonical;

    nl
    }
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

/// The bits of one side of an assignment, left index first: names expanded by their declared
/// range, a constant as `1'b0` / `1'b1` per bit (the reference binds them to its `zero_` / `one_`).
fn assign_bits(vals: &[String], decl_range: &std::collections::BTreeMap<String, (i64, i64)>) -> Vec<String> {
    vals.iter()
        .flat_map(|v| {
            if is_const(v) {
                const_bits(v).into_iter().map(|b| if b { "1'b1".to_string() } else { "1'b0".to_string() }).collect()
            } else {
                expand_bits(v, decl_range)
            }
        })
        .collect()
}

/// `lhs = rhs` as the reference's `mergeAssignNet` takes it: bit for bit when the two sides are
/// the same width, and NOTHING when they are not. `None` for a width mismatch.
fn assign_pairs(
    lhs: &[String],
    rhs: &[String],
    decl_range: &std::collections::BTreeMap<String, (i64, i64)>,
) -> Option<Vec<(String, String)>> {
    let (l, r) = (assign_bits(lhs, decl_range), assign_bits(rhs, decl_range));
    (l.len() == r.len()).then(|| l.into_iter().zip(r).collect())
}

pub fn load(path: &str) -> Result<Netlist, NetlistError> {
    let mut d = load_all(path)?;
    Ok(d.modules.swap_remove(d.top))
}

/// [`load`], keeping every module — see [`Design`]. STREAMED: read through a buffer, and
/// inflated as it is read when it is gzip (with the `gzip` feature), as the reference reads
/// through `igzstream`. A byte that is not UTF-8 is read lossily.
pub fn load_all(path: &str) -> Result<Design, NetlistError> {
    Ok(load_compact(path)?.to_design())
}

/// [`load_all`] in the COMPACT form — see [`CompactDesign`]. Streamed, as [`load_all`] is.
pub fn load_compact(path: &str) -> Result<CompactDesign, NetlistError> {
    let err = |e: String| NetlistError(format!("{path}: {e}"));
    parse_compact_reader(open(path).map_err(|e| err(e.to_string()))?).map_err(|e| err(e.0))
}

/// A netlist file as a buffered byte stream, inflated on the fly when it is gzip.
fn open(path: &str) -> std::io::Result<Box<dyn std::io::BufRead>> {
    use std::io::BufRead;
    let mut f = std::io::BufReader::with_capacity(1 << 16, std::fs::File::open(path)?);
    if f.fill_buf()?.starts_with(&[0x1f, 0x8b]) {
        return gz_stream(f);
    }
    Ok(Box::new(f))
}

/// A netlist file's whole text, inflated and read LOSSILY — for a caller that wants the text.
/// The readers here do not; they stream.
///
/// ⛔ `read_to_string` refused both outright. Netlists are shipped gzipped, and a comment or a
/// string written in a legacy code page (Windows-1252) is enough to make a whole file "not
/// UTF-8" although every identifier in it is ASCII.
pub fn read_text(path: &str) -> Result<String, NetlistError> {
    let err = |e: std::io::Error| NetlistError(format!("{path}: {e}"));
    let mut bytes = Vec::new();
    std::io::Read::read_to_end(&mut open(path).map_err(err)?, &mut bytes).map_err(err)?;
    Ok(match String::from_utf8(bytes) {
        Ok(s) => s,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    })
}

#[cfg(feature = "gzip")]
fn gz_stream<R: std::io::BufRead + 'static>(r: R) -> std::io::Result<Box<dyn std::io::BufRead>> {
    Ok(Box::new(std::io::BufReader::with_capacity(1 << 16, GzReader::new(r))))
}

#[cfg(not(feature = "gzip"))]
fn gz_stream<R: std::io::BufRead + 'static>(_: R) -> std::io::Result<Box<dyn std::io::BufRead>> {
    Err(std::io::Error::other("gzip-compressed; build vyges-loom with the `gzip` feature to read it"))
}

/// A gzip stream inflated as it is read: every member, concatenated (RFC 1952; `bgzip` and
/// `cat a.gz b.gz` write more than one). The inflater's state is all that is held.
#[cfg(feature = "gzip")]
struct GzReader<R: std::io::BufRead> {
    r: R,
    st: Box<miniz_oxide::inflate::stream::InflateState>,
    state: Gz,
}

#[cfg(feature = "gzip")]
#[derive(Clone, Copy, PartialEq, Eq)]
enum Gz {
    Header,
    Body,
    Done,
}

#[cfg(feature = "gzip")]
impl<R: std::io::BufRead> GzReader<R> {
    fn new(r: R) -> Self {
        let st = miniz_oxide::inflate::stream::InflateState::new_boxed(miniz_oxide::DataFormat::Raw);
        GzReader { r, st, state: Gz::Header }
    }

    /// Exactly `n` bytes from the stream, or an error at its end.
    fn take(&mut self, n: usize) -> std::io::Result<Vec<u8>> {
        let mut v = vec![0u8; n];
        std::io::Read::read_exact(&mut self.r, &mut v)?;
        Ok(v)
    }

    /// One member's header; `false` at a clean end of the file (or trailing non-gzip bytes).
    fn header(&mut self) -> std::io::Result<bool> {
        if !self.r.fill_buf()?.starts_with(&[0x1f, 0x8b]) {
            return Ok(false);
        }
        let h = self.take(10)?;
        let flg = h[3];
        if flg & 0x04 != 0 {
            let x = self.take(2)?;
            self.take(u16::from_le_bytes([x[0], x[1]]) as usize)?;
        }
        for flag in [0x08, 0x10] {
            if flg & flag != 0 {
                let mut z = Vec::new();
                std::io::BufRead::read_until(&mut self.r, 0, &mut z)?;
            }
        }
        if flg & 0x02 != 0 {
            self.take(2)?;
        }
        self.st.reset(miniz_oxide::DataFormat::Raw);
        Ok(true)
    }
}

#[cfg(feature = "gzip")]
impl<R: std::io::BufRead> std::io::Read for GzReader<R> {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        use miniz_oxide::{MZFlush, MZStatus};
        let bad = |m: String| std::io::Error::new(std::io::ErrorKind::InvalidData, m);
        loop {
            match self.state {
                Gz::Done => return Ok(0),
                Gz::Header => self.state = if self.header()? { Gz::Body } else { Gz::Done },
                Gz::Body => {
                    let input = self.r.fill_buf()?;
                    let eof = input.is_empty();
                    let res = miniz_oxide::inflate::stream::inflate(&mut self.st, input, out, MZFlush::None);
                    self.r.consume(res.bytes_consumed);
                    match res.status {
                        Ok(MZStatus::StreamEnd) => {
                            self.take(8)?; // CRC-32 and size; then perhaps another member
                            self.state = Gz::Header;
                        }
                        Ok(_) => {}
                        Err(miniz_oxide::MZError::Buf) if !eof => {}
                        Err(e) => return Err(bad(format!("gzip: {e:?}"))),
                    }
                    if res.bytes_written > 0 {
                        return Ok(res.bytes_written);
                    }
                    if eof && res.bytes_consumed == 0 && self.state == Gz::Body {
                        return Err(bad("truncated gzip stream".into()));
                    }
                }
            }
        }
    }
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

    /// Rule (the reference lexer's `^[ \t]*`.*{EOL}`): a line that STARTS, after blanks, with a
    /// compiler directive is dropped whole — not evaluated, so both arms of an `` `ifdef `` are
    /// read. A backtick later in a line is not a directive line.
    #[test]
    fn a_directive_line_is_dropped_whole() {
        let src = "`timescale 1ns/1ps\nmodule top (input a, output y);\n  wire n;\n \t`default_nettype wire\n  INV u0 (.A(a), .Y(n));\n`ifdef USE_POWER_PINS\n  BUF u1 (.A(n),\n`else\n  .VPWR(vdd),\n`endif\n  .Y(y));\nendmodule\n";
        let nl = parse(src).unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["u0", "u1"], "the directive line before u0 is not a `wire` declaration");
        assert_eq!(nl.insts[1].conns.len(), 3, "both arms of the ifdef are read: A, VPWR and Y");
    }

    /// Rule (the reference lexer's `{ID_TOKEN}("."{ID_TOKEN})*`): an identifier, a `.` and an
    /// identifier with nothing between them are ONE name; a `.` before a pin name is not.
    #[test]
    fn a_hierarchical_name_is_one_identifier() {
        let src = "module top (input a, output y);\n  INV u0 (.A(u1.n), .Y(y));\n  BUF u2 (.A(a),.Y(u3.m_2));\nendmodule";
        let nl = parse(src).unwrap();
        let c = |k: usize| nl.insts[k].conns.iter().map(|(p, n)| (p.as_str(), n.as_str())).collect::<Vec<_>>();
        assert_eq!(c(0), [("A", "u1.n"), ("Y", "y")]);
        assert_eq!(c(1), [("A", "a"), ("Y", "u3.m_2")]);
        // a name ending at the end of the input before a `.`, and a number before one
        let mut arena = Vec::new();
        let mut lx = Lexer::new("x. 1.5".as_bytes());
        let mut toks = Vec::new();
        while let Some((r, _)) = lx.next_into(&mut arena).unwrap() {
            toks.push(String::from_utf8_lossy(&arena[r]).into_owned());
        }
        assert_eq!(toks, ["x", ".", "1", ".", "5"]);
    }

    /// Rule (the reference reader's `VerilogBusNetNameIterator`): a bus's bits run from the LEFT
    /// index to the right one as declared — `[0:3]` is 0,1,2,3 — for a declaration, a part-select
    /// and a whole-bus connection alike; a port's range is kept over a later `wire` range
    /// (`parseDcl`). The first bit connected lands on the pin's most significant bit.
    #[test]
    fn a_bus_runs_in_its_declared_order() {
        let src = "module top (x, y);\n  input [0:3] x;\n  output [3:0] y;\n  wire [0:1] w;\n  wire [7:0] x;\n  BUF4 u0 (.A(x), .Y(y));\n  BUF2 u1 (.A(w), .Y(y[1:0]));\n  BUF2 u2 (.A(x[2:3]), .Y(w));\nendmodule";
        let nl = parse(src).unwrap();
        assert_eq!(nl.inputs, ["x[0]", "x[1]", "x[2]", "x[3]"]);
        assert_eq!(nl.outputs, ["y[3]", "y[2]", "y[1]", "y[0]"]);
        let c = |k: usize| nl.insts[k].conns.iter().map(|(p, n)| format!("{p}={n}")).collect::<Vec<_>>().join(" ");
        assert_eq!(c(0), "A[3]=x[0] A[2]=x[1] A[1]=x[2] A[0]=x[3] Y[3]=y[3] Y[2]=y[2] Y[1]=y[1] Y[0]=y[0]");
        assert_eq!(c(1), "A[1]=w[0] A[0]=w[1] Y[1]=y[1] Y[0]=y[0]");
        assert_eq!(c(2), "A[1]=x[2] A[0]=x[3] Y[1]=w[0] Y[0]=w[1]");
        // ANSI header ports the same way
        let nl = parse("module m (input [1:3] a, output z);\n  AND3 g (.A(a), .Y(z));\nendmodule").unwrap();
        assert_eq!(nl.inputs, ["a[1]", "a[2]", "a[3]"]);
        assert_eq!(nl.insts[0].conns[0], ("A[2]".to_string(), "a[1]".to_string()));
    }

    /// Rule (the reference grammar's `dcl_type`): `tri`, `wand` and `wor` declare nets with a
    /// range, as `wire` does.
    #[test]
    fn tri_wand_and_wor_declare_a_range() {
        let src = "module top (output [1:0] y);\n  tri [1:0] t;\n  wand [1:0] a;\n  wor [1:0] o;\n  B u0 (.A(t), .B(a), .C(o), .Y(y));\nendmodule";
        let nl = parse(src).unwrap();
        let pins: Vec<&str> = nl.insts[0].conns.iter().map(|(p, _)| p.as_str()).collect();
        assert_eq!(pins, ["A[1]", "A[0]", "B[1]", "B[0]", "C[1]", "C[0]", "Y[1]", "Y[0]"]);
    }

    /// Rule (the reference's `mergeAssignNet`): an assignment merges its two sides BIT FOR BIT —
    /// buses, part-selects and concatenations expanded, a constant's bits tied — and merges
    /// nothing when the widths differ (warning 203). A statement may hold several, and a net
    /// declaration's `= rhs` is one too.
    #[test]
    fn an_assign_merges_bit_for_bit() {
        let src = "module top (x, y, z);\n  input [0:1] x;\n  output [1:0] y;\n  output [2:0] z;\n  wire [1:0] w;\n  wire [1:0] v = w;\n  wire s = x[0], q;\n  assign w = x, z = {y[0], 2'b10};\n  assign y = {q, s};\n  assign q = z;\nendmodule";
        let nl = parse(src).unwrap();
        let a: Vec<String> = nl.aliases.iter().map(|(l, r)| format!("{l}={r}")).collect();
        assert_eq!(
            a,
            ["v[1]=w[1]", "v[0]=w[0]", "s=x[0]", "w[1]=x[0]", "w[0]=x[1]", "z[2]=y[0]", "z[1]=1'b1", "z[0]=1'b0", "y[1]=q", "y[0]=s"]
        );
        assert_eq!(nl.skipped_assigns, 1, "q (1 bit) = z (3 bits) is not merged");
        assert!(!nl.aliases.iter().any(|(l, _)| l == "q" || l.starts_with("q[")), "q is not declared [1:0]");
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

    /// The reader STREAMS: fed a few bytes at a time, so that every lexer state — a `/` waiting
    /// for its next byte, a comment, an escaped identifier, a token — is cut by a buffer refill,
    /// it reads exactly what a whole-text read does.
    #[test]
    fn a_streamed_read_is_the_whole_read() {
        let src = "// head\nmodule leaf (input a, output y); /* INV ghost (.A(a)); */\n\
                   INV \\u0.x[1] (.A(a), .Y(y)); // tail\nendmodule\n\
                   module top (input a, output [1:0] y); wire w/*c*/; leaf l (.a(a), .y(w));\n\
                   BUF b (.A({w, 1'b1}), .X(y)); endmodule/";
        let whole = parse_all(src).unwrap();
        let key = |d: &Design| {
            d.modules.iter().map(|m| format!("{}|{:?}|{:?}|{:?}", m.module, m.inputs, m.outputs,
                m.insts.iter().map(|i| (&i.cell, &i.name, &i.conns, &i.ties)).collect::<Vec<_>>())).collect::<Vec<_>>()
        };
        assert_eq!(whole.module_names(), ["leaf", "top"]);
        assert!(whole.escaped_names().contains("u0.x[1]"));
        for cap in [1, 2, 3, 7] {
            let d = parse_reader(std::io::BufReader::with_capacity(cap, src.as_bytes())).unwrap();
            assert_eq!(key(&d), key(&whole), "buffer of {cap} byte(s)");
            assert_eq!(d.escaped_names(), whole.escaped_names());
        }
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
        // and inside a NAME, the byte becomes U+FFFD and the rest of the module still reads
        let named = b"module top (input a, output y);\n  INV u\x920 (.A(a), .Y(y));\nendmodule\n";
        let d = parse_reader(&named[..]).unwrap();
        assert_eq!(d.top().insts[0].name, "u\u{FFFD}0");
        assert_eq!(d.top().insts[0].conns.len(), 2);
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

#[cfg(test)]
mod compact_tests {
    use super::*;

    /// A name is stored once and found again; the index survives growing past many resizes.
    #[test]
    fn the_pool_stores_each_name_once() {
        let mut p = StrPool::default();
        let a = p.intern("clk");
        assert_eq!(p.intern("clk"), a);
        let e = p.intern("");
        assert_eq!(p.get(e), "");
        let syms: Vec<Sym> = (0..50_000).map(|k| p.intern(&format!("n{k}"))).collect();
        assert_eq!(p.len(), 50_002);
        for (k, s) in syms.iter().enumerate().step_by(997) {
            assert_eq!(p.get(*s), format!("n{k}"));
            assert_eq!(p.find(&format!("n{k}")), Some(*s));
        }
        assert_eq!(p.find("absent"), None);
        assert_eq!(p.get(a), "clk");
    }

    /// The compact form holds exactly what the owned one does — the owned one is made from it,
    /// so what is checked here is the compact accessors.
    #[test]
    fn the_compact_form_reads_like_the_owned_one() {
        let src = "module leaf (input a, output y); INV u0 (.A(a), .Y(y)); endmodule\n\
                   module top (input a, output [1:0] y); wire w; assign y[0] = w; \
                   leaf l (.a(a), .y(w)); BUF b (.A({w, 1'b1}), .X(y[1])); endmodule";
        let c = parse_compact(src).unwrap();
        let d = c.to_design();
        assert_eq!(c.str(c.top().name), "top");
        let leaf = c.module("leaf").expect("leaf");
        assert_eq!(c.str(leaf.insts[0].cell), "INV");
        for (cm, nm) in c.modules.iter().zip(&d.modules) {
            assert_eq!(cm.insts.len(), nm.insts.len());
            for (ci, ni) in cm.insts.iter().zip(&nm.insts) {
                assert_eq!(c.str(ci.name), ni.name);
                let conns: Vec<(String, String)> =
                    cm.conns(ci).iter().map(|(p, n)| (c.str(*p).to_string(), c.str(*n).to_string())).collect();
                assert_eq!(conns, ni.conns);
                let ties: Vec<(String, bool)> = cm.ties(ci).iter().map(|(p, v)| (c.str(*p).to_string(), *v)).collect();
                assert_eq!(ties, ni.ties);
            }
        }
        let b = &c.top().insts[1];
        assert_eq!(c.top().ties(b), [(c.pool.find("A[0]").unwrap(), true)]);
        // the alias renamed `w` to the port it drives, in the compact form as in the owned one
        assert_eq!(d.top().canonical.get("w").map(String::as_str), Some("y[0]"));
    }
}
