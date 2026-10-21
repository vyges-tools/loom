//! Structural (gate-level) Verilog reader — the connectivity STA and placement need.
//!
//! Reads every module in a file — header, `input`/`output`/`inout` ports (scalar and bussed),
//! net declarations (`wire`/`tri`/`wand`/`wor`, `supply0`/`supply1`), `assign` statements and cell
//! or module instances, by name (`.PIN(net)`) or by position — then picks the top module (the one
//! nothing instantiates).
//!
//! 🔑 **The rules follow the reference reader, not a reading of the LRM**, and each is pinned by a
//! test at its site:
//! - a compiler-directive line (`` `timescale ``, `` `define ``) is dropped whole;
//! - `a.b` is one hierarchical identifier, and an escaped identifier is stored canonical;
//! - a bus expands in its DECLARED order (`[0:7]` runs 0 to 7), and a port keeps its own range
//!   over a later net declaration of the same name;
//! - `assign` merges nets bit for bit (a width mismatch is counted in `skipped_assigns`, not
//!   merged), and a constant on a pin is recorded in `Inst::ties`, not as a connection;
//! - a module defined twice keeps its LAST definition, and a duplicated instance name is renamed
//!   `name_k`.
//!
//! `parameter` and behavioural code are skipped. Pure std — fully unit-tested offline.

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
    /// The port bits, in the order the module's HEADER names them, by declared direction.
    ///
    /// 🔑 **The header says what is a port** — the reference's `makeCellPorts`: a declared
    /// `input` the header does not list is a net (warning 197), `{a, b}` in the header makes no
    /// port (2721), `y[1]` makes a port NAMED `y[1]`, and `i[1:0]` exposes bits 1..0 of `i`.
    /// Ports were taken from the declarations until 2026-10-21, in declaration order.
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
    /// Pin bits of instances of modules DEFINED in the file left unconnected, as the reference
    /// leaves them: the module has no such port (warning 201), or the port is not the
    /// connection's width (warning 200). A cell that is not defined here is never checked.
    pub unbound_pins: usize,
    /// Nets a `supply0` (false) / `supply1` (true) declaration holds constant, by the name as
    /// declared — the reference's `addConstantNet`. Pins stay connected to the net; this is
    /// what says its value.
    ///
    /// ⚠️ The reference binds the declared NAME, so `supply0 [3:0] g;` marks the net `g`, not
    /// its bits; transcribed as it does.
    pub constants: Vec<(String, bool)>,
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
    /// Header ports with no `input`/`output`/`inout` declaration (warning 166), or only a net
    /// one: the reference makes them ports with no direction, so they are in none of `inputs`,
    /// `outputs` or `inouts`. Header order.
    pub undirected: Vec<String>,
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
        if self.unbound_pins > 0 {
            notes.push(format!(
                "{} pin bit(s) of a module instance left unconnected (no such port, or a width \
                 mismatch), as the reference does",
                self.unbound_pins
            ));
        }
        if !self.undirected.is_empty() {
            notes.push(format!(
                "{} header port(s) with no direction declared — ports, but neither input nor output",
                self.undirected.len()
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
/// The same inside an escaped identifier, which ONLY whitespace ends — a `//` or `/*` in one is
/// part of its name. The reference's flex scanner takes the longest match from where it stands,
/// and from a backslash that is `\\[^ \t\r\n]+[\r\n\t ]`: a comment rule only competes at a token's
/// start. Ending the name at `//` read `wire \a//b ;` as `wire \a` and lost the rest of the line.
const STOP_ESCAPED: [bool; 256] = stop_table(true);

const fn stop_table(escaped: bool) -> [bool; 256] {
    let mut t = [false; 256];
    let mut i = 0;
    while i < 256 {
        let b = i as u8;
        t[i] = b.is_ascii_whitespace()
            || (!escaped && matches!(b, b'/' | b'\\' | b'(' | b')' | b';' | b',' | b'.' | b'[' | b']' | b'='));
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
                    // inside an escaped identifier only whitespace ends it
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
                // Comments — BOTH forms — end whatever ordinary token is open (an escaped one
                // never reaches here on a `/`: see STOP_ESCAPED).
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
    // a declared vector by its WHOLE name first: an escaped vector `\ti[0] ` is canonically
    // `ti[0]`, which would otherwise read as a bit-select of `ti`
    if let Some(&(m, l)) = decl_range.get(net) {
        return if m != l { bus_bits(net, m, l) } else { vec![net.to_string()] };
    }
    // the LAST bracket is the select: `ti[0][1:2]` is bits 1..2 of the vector `ti[0]`
    if let Some((base, rest)) = net.rsplit_once('[') {
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
    matches!(t.chars().next(), Some(c) if c.is_ascii_alphabetic() || c == '_' || c == '\\' || c == KEPT as char)
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

    /// Strips the [`KEPT`] tag from every name that carries it, each symbol keeping its index.
    fn untag_kept(&mut self) {
        let mut text = String::with_capacity(self.text.len());
        let mut ends = Vec::with_capacity(self.ends.len());
        for i in 0..self.ends.len() {
            let s = self.get(Sym(i as u32));
            text.push_str(s.strip_prefix(KEPT as char).unwrap_or(s));
            ends.push(text.len() as u64);
        }
        self.text = text;
        self.ends = ends;
        // the index again; a name that now spells another keeps both symbols, `find` the first
        let cap = self.table.len();
        self.table = vec![EMPTY; cap];
        let mask = cap - 1;
        for id in 0..self.ends.len() as u32 {
            let s = self.get(Sym(id));
            let mut h = fx_hash(s) as usize & mask;
            let mut dup = false;
            while self.table[h] != EMPTY {
                if self.get(Sym(self.table[h])) == s {
                    dup = true;
                    break;
                }
                h = (h + 1) & mask;
            }
            if !dup {
                self.table[h] = id;
            }
        }
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
    /// see [`Netlist::unbound_pins`]
    pub unbound_pins: usize,
    /// see [`Netlist::constants`]
    pub constants: Vec<(Sym, bool)>,
    /// written name -> canonical name — see [`Netlist::canonical`]
    pub canonical: Vec<(Sym, Sym)>,
    pub behavioural: usize,
    pub positional: usize,
    /// see [`Netlist::undirected`]
    pub undirected: Vec<Sym>,
    /// the module's ports as a PARENT sees them, in header order — see [`CellPort`]
    cell_ports: Vec<CellPort>,
}

/// One port of a module as the reference's `makeCellPorts` makes it from the header: a name, a
/// bit-select (a port NAMED `y[1]`), or a part-select's name (`i[1:0]` makes the port `i`, every
/// bit its declaration has); a `.X(expr)` entry makes a port per bit of `expr`, then the BUNDLE `X`
/// over them. Positional connections pair with these in this order.
#[derive(Debug, Clone)]
struct CellPort {
    name: Sym,
    /// in declared order, left index first; for a bundle, its members
    bits: Vec<Sym>,
    bundle: bool,
}

/// A port declaration's direction, as the reference's declaration map keeps it (`parseDcl`): the
/// first declaration of a name wins, except that a `wire` (or `reg`, `tri`, a supply) is replaced
/// by a later direction — a net type before or after `input` only modifies it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PortDir {
    Input,
    Output,
    Inout,
    Internal,
}

fn declare(dirs: &mut std::collections::HashMap<String, PortDir>, name: &str, d: PortDir) {
    match dirs.get(name) {
        Some(&PortDir::Internal) if d != PortDir::Internal => {
            dirs.insert(name.to_string(), d);
        }
        Some(_) => {}
        None => {
            dirs.insert(name.to_string(), d);
        }
    }
}

/// One entry of a module header's port list (the reference grammar's `port`).
#[derive(Debug, Clone)]
enum HeaderPort {
    /// `a`, `y[1]`, or `i[1:0]`: the port `port`, and the expression the entry names — its bits
    /// are the pins the module has as a TOP (`linkNetwork` looks each up)
    Named { port: String, refs: String },
    /// `.X(expr)`: a port per bit of `expr`, then the bundle `X`
    Bundle { port: String, refs: Vec<String> },
    /// `{a, b}`, or anything else with no port identifier: no port (warning 2721)
    Unnamed { refs: Vec<String> },
}

/// The pin name a POSITIONAL connection carries until the file is read and it is bound: the
/// expression's place `k` in the list and the bit's place `b` (`s` its bit, once expanded) in the
/// expression. A byte no Verilog source holds starts it; none survives the read.
const POS_TAG: char = '\u{2}';

fn pos_tag(pool: &mut StrPool, k: u32, b: u32, s: Option<u32>) -> Sym {
    match s {
        Some(s) => pool.intern(&format!("{POS_TAG}{k}.{b}.{s}")),
        None => pool.intern(&format!("{POS_TAG}{k}.{b}")),
    }
}

/// The pin name a NAMED connection carries while what it names is not declared yet: its place in
/// the connection, then the pin. Expanded, and the pin's bits named, at [`ModuleReader::finish`].
const NAMED_TAG: char = '\u{3}';

fn named_tag(pool: &mut StrPool, order: u32, pin: &str) -> Sym {
    pool.intern(&format!("{NAMED_TAG}{order}{NAMED_TAG}{pin}"))
}

fn read_named_tag(p: &str) -> Option<(u32, &str)> {
    let (order, pin) = p.strip_prefix(NAMED_TAG)?.split_once(NAMED_TAG)?;
    Some((order.parse().ok()?, pin))
}

/// `(k, b, s)` of a positional tag, or `None` for a real pin name.
fn read_pos_tag(p: &str) -> Option<(u32, u32, u32)> {
    let mut it = p.strip_prefix(POS_TAG)?.split('.').map(|x| x.parse::<u32>());
    Some((it.next()?.ok()?, it.next()?.ok()?, it.next().unwrap_or(Ok(0)).ok()?))
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
            unbound_pins: self.unbound_pins,
            constants: self.constants.iter().map(|(s, v)| (st(s), *v)).collect(),
            canonical: self.canonical.iter().map(|(a, b)| (st(a), st(b))).collect(),
            behavioural: self.behavioural,
            positional: self.positional,
            undirected: self.undirected.iter().map(st).collect(),
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

    /// The module DEFINED under `name`, if the file defines it (the last definition, if twice).
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
    /// found here is a leaf — a library cell or a black box. A name defined twice keeps only its
    /// last definition, as the reference does.
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
/// Marks an escaped name the statement parser would otherwise read as syntax — see [`keep_escape`].
const KEPT: u8 = 0x01;

/// An escaped identifier is a NAME whatever it spells (the reference lexer returns it as an ID):
/// `\output ` is a port called `output`, `\; ` a net, `\1x ` an instance. The canonical text the
/// lexer hands over would read as a keyword, punctuation, a number or a constant, so such a token
/// is tagged with [`KEPT`] (a byte no Verilog source holds) and parsed as the name it is; the
/// tag is stripped from the pool once the file is read ([`StrPool::untag_kept`]). `tok` is the
/// last token in `arena`, so the insert moves nothing else.
fn keep_escape(arena: &mut Vec<u8>, tok: std::ops::Range<usize>, kept: &mut bool) -> std::ops::Range<usize> {
    let w = &arena[tok.clone()];
    let syntax = match w.first() {
        Some(c) if c.is_ascii_alphabetic() || *c == b'_' || *c == b'\\' => {
            w.contains(&b'\'') || std::str::from_utf8(w).is_ok_and(|t| is_keyword(t) || RESERVED.contains(&t))
        }
        _ => true,
    };
    if !syntax {
        return tok;
    }
    arena.insert(tok.start, KEPT);
    *kept = true;
    tok.start..tok.end + 1
}

/// Words the statement parser acts on beyond [`is_keyword`].
const RESERVED: &[&str] = &[
    "signed", "logic", "task", "function", "initial", "generate", "always", "always_ff",
    "always_comb", "always_latch",
];

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
    // an escaped name that reads as syntax was tagged (see `keep_escape`); untagged at the end
    let mut kept = false;
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
            let t = if esc { keep_escape(&mut arena, t, &mut kept) } else { t };
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
                    let t = if esc { keep_escape(&mut arena, t, &mut kept) } else { t };
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
    // A module DEFINED TWICE keeps its LAST definition: the reference's `makeModule` deletes the
    // earlier cell and makes the new one. Kept both, the first answered every lookup. (Once, at
    // the end: per module it was quadratic in the module count.)
    let mut last: std::collections::HashMap<Sym, usize> = std::collections::HashMap::with_capacity(mods.len());
    for (k, m) in mods.iter().enumerate() {
        last.insert(m.name, k);
    }
    if last.len() < mods.len() {
        let mut k = 0;
        mods.retain(|m| {
            k += 1;
            last[&m.name] == k - 1
        });
    }
    if mods.is_empty() {
        return Err(NetlistError("no module found".into()));
    }
    bind_defined_child_pins(&mut mods, &mut pool);

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
    if kept {
        pool.untag_kept();
    }
    Ok(CompactDesign { pool, modules: mods, top, escaped, by_name })
}

/// The pins of every instance of a module DEFINED in the file, bound to its ports as the reference
/// binds them — once the whole file is read, since a child may be defined after its parent.
///
/// NAMED pins (`makeNamedInstPins`):
/// - a pin takes the connection's bits in the CHILD's declared order: the child port's members
///   (from its left index to its right) each pair with the connection's next bit. At the instance
///   the bits were named as a Liberty bus is, the first connection bit on the highest index
///   (`P[k-1-i]`); that is undone here, and a one-bit pin takes the child's own bit name;
/// - a pin the child has no port for (warning 201), or whose width is not the connection's
///   (warning 200), is left UNCONNECTED, and counted in [`CompactModule::unbound_pins`].
///
/// A pin whose bits are ambiguous — an escaped scalar `\foo[3] ` canonicalises to the name of a
/// bit of the bus `foo` — is left as written.
///
/// POSITIONAL values (`makeOrderedInstPins`) pair with the child's ports in HEADER order — the
/// k-th value with the k-th port, bit by bit, the port's bits from its left index. A value whose
/// width is not its port's (warning 202) connects nothing; a bundle port (`.X(a, b)` in the
/// header) cannot be connected by position (2723); values past the last port connect nothing.
/// Every other cell's positional pins get the empty name (see [`Netlist::positional`]).
fn bind_defined_child_pins(mods: &mut [CompactModule], pool: &mut StrPool) {
    use std::collections::HashMap;
    fn split_bit(s: &str) -> Option<(&str, usize)> {
        let (base, rest) = s.rsplit_once('[')?;
        Some((base, rest.strip_suffix(']')?.parse().ok()?))
    }
    // the bits of each pin, by its name: (j of `P[j]`, or None for a scalar `P`) — a well-formed
    // pin is one scalar, or bits 0..k-1 once each
    fn shape(bits: &[Option<usize>]) -> Option<usize> {
        if let [None] = bits {
            return Some(1);
        }
        let mut js: Vec<usize> = bits.iter().copied().collect::<Option<Vec<_>>>()?;
        js.sort_unstable();
        js.iter().enumerate().all(|(i, &j)| i == j).then_some(js.len())
    }
    let key = |pool: &StrPool, p: Sym| -> (String, Option<usize>) {
        match split_bit(pool.get(p)) {
            Some((base, j)) => (base.to_string(), Some(j)),
            None => (pool.get(p).to_string(), None),
        }
    };
    // each defined module's ports, in header order, and by name
    let ports: HashMap<Sym, (Vec<CellPort>, HashMap<Sym, usize>)> = mods
        .iter()
        .map(|m| (m.name, (m.cell_ports.clone(), m.cell_ports.iter().enumerate().map(|(k, p)| (p.name, k)).collect())))
        .collect();
    let none = pool.intern("");
    for m in mods.iter_mut() {
        let mut drop_conn = vec![false; m.conn_table.len()];
        let mut drop_tie = vec![false; m.tie_table.len()];
        let mut dropped = 0usize;
        for x in 0..m.insts.len() {
            let (c, t) = (m.insts[x].conns, m.insts[x].ties);
            let (c, t) = (c.0 as usize..c.1 as usize, t.0 as usize..t.1 as usize);
            // positional bits: (k, b, s) -> (table, index)
            let mut pos: Vec<((u32, u32, u32), bool, usize)> = Vec::new();
            for k in c.clone() {
                if let Some(tag) = read_pos_tag(pool.get(m.conn_table[k].0)) {
                    pos.push((tag, true, k));
                }
            }
            for k in t.clone() {
                if let Some(tag) = read_pos_tag(pool.get(m.tie_table[k].0)) {
                    pos.push((tag, false, k));
                }
            }
            let Some((child, by_name)) = ports.get(&m.insts[x].cell) else {
                for &(_, conn, k) in &pos {
                    if conn { m.conn_table[k].0 = none } else { m.tie_table[k].0 = none }
                }
                continue;
            };
            // an instance connects by position or by name, never both
            if !pos.is_empty() {
                pos.sort_unstable();
                let mut at = 0;
                let mut port_iter = child.iter();
                while at < pos.len() {
                    let k = pos[at].0 .0;
                    let end = at + pos[at..].iter().take_while(|e| e.0 .0 == k).count();
                    let value = &pos[at..end];
                    match port_iter.next() {
                        Some(port) if !port.bundle && port.bits.len() == value.len() => {
                            for (j, &(_, conn, idx)) in value.iter().enumerate() {
                                if conn { m.conn_table[idx].0 = port.bits[j] } else { m.tie_table[idx].0 = port.bits[j] }
                            }
                        }
                        _ => {
                            for &(_, conn, idx) in value {
                                if conn { drop_conn[idx] = true } else { drop_tie[idx] = true }
                            }
                            dropped += value.len();
                        }
                    }
                    at = end;
                }
                continue;
            }
            // (table, index) of every bit of each named pin
            let mut pins: HashMap<String, Vec<(bool, usize, Option<usize>)>> = HashMap::new();
            for k in c.clone() {
                let p = m.conn_table[k].0;
                if !pool.get(p).is_empty() {
                    let (name, j) = key(pool, p);
                    pins.entry(name).or_default().push((true, k, j));
                }
            }
            for k in t.clone() {
                let p = m.tie_table[k].0;
                if !pool.get(p).is_empty() {
                    let (name, j) = key(pool, p);
                    pins.entry(name).or_default().push((false, k, j));
                }
            }
            for (name, bits) in pins {
                let js: Vec<Option<usize>> = bits.iter().map(|b| b.2).collect();
                let Some(width) = shape(&js) else { continue };
                // a one-bit pin whose base names no port may be an escaped scalar, `\\wi[0] `: its
                // whole text is the port
                let whole = || if let [(conn, k, _)] = bits[..] { Some(if conn { m.conn_table[k].0 } else { m.tie_table[k].0 }) } else { None };
                let found = pool.find(&name).and_then(|s| by_name.get(&s)).or_else(|| whole().and_then(|s| by_name.get(&s)));
                let port = match found.map(|&k| &child[k]) {
                    Some(port) if !port.bundle && port.bits.len() == width => port,
                    // no such port, or not the connection's width: the reference connects nothing
                    _ => {
                        for &(conn, k, _) in &bits {
                            if conn { drop_conn[k] = true } else { drop_tie[k] = true }
                        }
                        dropped += bits.len();
                        continue;
                    }
                };
                for &(conn, k, j) in &bits {
                    let to = if width == 1 { port.bits[0] } else { port.bits[width - 1 - j.unwrap_or(0)] };
                    if conn { m.conn_table[k].0 = to } else { m.tie_table[k].0 = to }
                }
            }
        }
        if dropped > 0 {
            m.unbound_pins += dropped;
            let (old_c, old_t) = (std::mem::take(&mut m.conn_table), std::mem::take(&mut m.tie_table));
            for x in m.insts.iter_mut() {
                let c0 = m.conn_table.len() as u32;
                m.conn_table.extend((x.conns.0..x.conns.1).filter(|&k| !drop_conn[k as usize]).map(|k| old_c[k as usize]));
                let t0 = m.tie_table.len() as u32;
                m.tie_table.extend((x.ties.0..x.ties.1).filter(|&k| !drop_tie[k as usize]).map(|k| old_t[k as usize]));
                x.conns = (c0, m.conn_table.len() as u32);
                x.ties = (t0, m.tie_table.len() as u32);
            }
        }
        // what is still by position, bit by bit, once bound
        m.positional = m.conn_table.iter().filter(|(p, _)| *p == none).count();
    }
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
    /// the header's port list, in order — what the ports ARE (see [`Netlist::inputs`])
    header: Vec<HeaderPort>,
    /// each declared name's direction, by the reference's rule — see [`PortDir`]
    dirs: std::collections::HashMap<String, PortDir>,
    /// an instance connects by position: its connections are expanded at [`ModuleReader::finish`]
    pos_tagged: bool,
    /// a named connection waits for a later declaration — see [`named_tag`]
    named_deferred: bool,
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
                // the entries, split at the commas between them (not those inside `(…)` or `{…}`)
                let mut entries: Vec<&[&str]> = Vec::new();
                let (mut s0, mut depth, mut brace) = (i, 0i32, 0i64);
                while i < n {
                    match t[i] {
                        "(" => depth += 1,
                        ")" if depth == 0 => break,
                        ")" => depth -= 1,
                        "," if depth == 0 && brace == 0 => {
                            entries.push(&t[s0..i]);
                            s0 = i + 1;
                        }
                        tok => brace += tok.matches('{').count() as i64 - tok.matches('}').count() as i64,
                    }
                    i += 1;
                }
                entries.push(&t[s0..i.min(n)]);
                i += 1; // past `)`
                let dirs = &mut self.dirs;
                let header = &mut self.header;
                // An ANSI entry carries its direction, type and range; a bare name after it takes
                // the same ones (`input [3:0] a, b`)
                let (mut dir, mut range): (Option<PortDir>, Option<(i64, i64)>) = (None, None);
                for e in entries.into_iter().filter(|e| !e.is_empty()) {
                    let mut k = 0;
                    let d = match e[0] {
                        "input" => Some(PortDir::Input),
                        "output" => Some(PortDir::Output),
                        "inout" => Some(PortDir::Inout),
                        _ => None,
                    };
                    if d.is_some() {
                        (dir, range, k) = (d, None, 1);
                        while k < e.len() && matches!(e[k], "wire" | "reg" | "logic" | "signed") {
                            k += 1;
                        }
                        if e.get(k) == Some(&"[") {
                            let close = e.iter().position(|x| *x == "]").unwrap_or(e.len());
                            let spec: String = e[k + 1..close].concat();
                            range = spec.split_once(':').and_then(|(h, l)| {
                                Some((h.trim().parse::<i64>().ok()?, l.trim().parse::<i64>().ok()?))
                            });
                            k = close + 1;
                        }
                    }
                    if let Some(d) = dir {
                        if let Some(&name) = e.get(k).filter(|x| is_ident(x) && !is_keyword(x)) {
                            port_decl.insert(name.to_string());
                            if let Some(r) = range {
                                decl_range.insert(name.to_string(), r);
                            }
                            declare(dirs, name, d);
                            header.push(HeaderPort::Named { port: name.to_string(), refs: name.to_string() });
                        }
                        continue;
                    }
                    // A non-ANSI entry names a port; the `input`/`output` statements below give
                    // it its direction and range (the reference grammar's `port`)
                    let entry = match e {
                        [".", x, "(", rest @ .., ")"] => {
                            let mut j = 0;
                            HeaderPort::Bundle { port: x.to_string(), refs: read_conn_value(rest, &mut j) }
                        }
                        [name] if is_ident(name) => HeaderPort::Named { port: name.to_string(), refs: name.to_string() },
                        [name, "[", sel @ .., "]"] if is_ident(name) => {
                            let sel = sel.concat();
                            let refs = format!("{name}[{sel}]");
                            // a part-select is the port `name`; a bit-select is a port NAMED `name[k]`
                            let port = if sel.contains(':') { name.to_string() } else { refs.clone() };
                            HeaderPort::Named { port, refs }
                        }
                        _ => {
                            let mut j = 0;
                            HeaderPort::Unnamed { refs: read_conn_value(e, &mut j) }
                        }
                    };
                    header.push(entry);
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
                    // a net type after the direction (`output reg q;`, `input wire a;`) is not a
                    // name; read as one, it made a port called `reg`
                    "," | "wire" | "reg" | "logic" | "signed" | "tri" | "wand" | "wor" | "supply0" | "supply1" => {}
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
                        if let Some(r) = range {
                            decl_range.insert(tok.to_string(), r);
                        }
                        names.push(tok.to_string());
                    }
                }
                *i += 1;
            }
            names
        };

        while i < n {
            match t[i] {
                "endmodule" => break,
                // A direction and a range for each name; whether it is a PORT is the header's to
                // say, once the module is read (`finish`). The range is kept for `expand_bits` too,
                // when an instance below connects the whole bus.
                "input" => {
                    i += 1;
                    for x in read_names(&mut i, decl_range, port_decl) {
                        declare(&mut self.dirs, &x, PortDir::Input);
                    }
                }
                "output" => {
                    i += 1;
                    for x in read_names(&mut i, decl_range, port_decl) {
                        declare(&mut self.dirs, &x, PortDir::Output);
                    }
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
                    for x in read_names(&mut i, decl_range, port_decl) {
                        declare(&mut self.dirs, &x, PortDir::Inout);
                    }
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
                            // `wire [3:0] w = …;` DECLARES NOTHING: the reference keeps only a
                            // named argument (`parseDcl`, `isNamed`), so `w` stays an undeclared
                            // one-bit net and the assignment's widths are 1 and the right side's
                            // (warning 203 for a wider one). A supply's constant still applies.
                            let declared = t.get(i + 1) != Some(&"=");
                            if declared && !matches!(kw, "parameter" | "localparam") {
                                declare(&mut self.dirs, t[i], PortDir::Internal);
                            }
                            if kw == "supply0" || kw == "supply1" {
                                nl.constants.push((pool.intern(t[i]), kw == "supply1"));
                            }
                            if let (true, Some(idx)) = (declared, &single) {
                                one_bit_vec.insert(t[i].to_string(), idx.clone());
                            }
                            if let (true, Some(r)) = (declared, range) {
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
                        let (mut pos_k, mut pos_b) = (0u32, 0u32);
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
                                    //
                                    // ⚠️ The width of a name is its DECLARATION's, and the reference
                                    // reads it at link time, so a declaration may follow the instance
                                    // (`sub u (.i(a)); input [3:0] a;`). A connection naming something
                                    // not declared yet is held as written (`named_tag`) and expanded
                                    // at `finish`.
                                    if nets.iter().any(|v| !is_const(v) && !v.contains('[') && !self.dirs.contains_key(v.as_str())) {
                                        let mut order = 0u32;
                                        for v in &nets {
                                            if is_const(v) {
                                                for bit in const_bits(v) {
                                                    nl.tie_table.push((named_tag(pool, order, &pin), bit));
                                                    order += 1;
                                                }
                                            } else {
                                                nl.conn_table.push((named_tag(pool, order, &pin), pool.intern(v)));
                                                order += 1;
                                            }
                                        }
                                        self.named_deferred = true;
                                        i += 1;
                                        continue;
                                    }
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
                                // Each value's bits are tagged with its place in the list and
                                // theirs in it (`pos_tag`), constants included: a module defined in
                                // the file binds them to its ports in order once the file is read
                                // (`makeOrderedInstPins`); any other cell gets the empty pin name.
                                _ if !named => {
                                    let tok = t[i];
                                    if tok == "," {
                                        if depth == 1 {
                                            (pos_k, pos_b) = (pos_k + 1, 0);
                                        }
                                        i += 1;
                                    } else if is_const(tok) || is_ident(tok) || tok.starts_with('{') {
                                        let mut j = i;
                                        for v in read_conn_value(t, &mut j) {
                                            if is_const(&v) {
                                                for bit in const_bits(&v) {
                                                    let p = pos_tag(pool, pos_k, pos_b, Some(0));
                                                    nl.tie_table.push((p, bit));
                                                    pos_b += 1;
                                                }
                                            } else {
                                                let p = pos_tag(pool, pos_k, pos_b, None);
                                                nl.conn_table.push((p, pool.intern(&v)));
                                                nl.positional += 1;
                                                pos_b += 1;
                                            }
                                        }
                                        self.pos_tagged = true;
                                        i = j.max(i + 1);
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
        let (dirs, decl_range) = (self.dirs, self.decl_range);
        // ---- the ports: what the HEADER makes (`makeCellPorts`), then the top's pins ----------
        //
        // `makeCellPort`: a declared name takes its declaration's direction and range; one with no
        // declaration is a scalar port with no direction (warning 166). A name listed twice is one
        // port (165); `{a, b}` makes none (2721).
        fn make_port(
            name: &str,
            dirs: &std::collections::HashMap<String, PortDir>,
            decl_range: &std::collections::BTreeMap<String, (i64, i64)>,
            bit_dir: &mut std::collections::HashMap<String, Option<PortDir>>,
            pool: &mut StrPool,
        ) -> CellPort {
            let dir = dirs.get(name).copied();
            let bits = match (dir, decl_range.get(name)) {
                (Some(_), Some(&(h, l))) => bus_bits(name, h, l),
                _ => vec![name.to_string()],
            };
            for b in &bits {
                bit_dir.insert(b.clone(), dir);
            }
            CellPort { name: pool.intern(name), bits: bits.iter().map(|b| pool.intern(b)).collect(), bundle: false }
        }
        let refs_bits = |refs: &[String]| -> Vec<String> {
            refs.iter().filter(|r| !is_const(r)).flat_map(|r| expand_bits(r, &decl_range)).collect()
        };
        let mut port_names: std::collections::HashSet<&str> = std::collections::HashSet::new();
        let mut bit_dir: std::collections::HashMap<String, Option<PortDir>> = std::collections::HashMap::new();
        let mut cell_ports: Vec<CellPort> = Vec::new();
        let mut members: Vec<String> = Vec::new();
        for h in &self.header {
            match h {
                HeaderPort::Named { port, .. } => {
                    if port_names.insert(port.as_str()) {
                        cell_ports.push(make_port(port, &dirs, &decl_range, &mut bit_dir, pool));
                    }
                }
                // a port per bit of the expression, then the bundle over them
                HeaderPort::Bundle { port, refs } => {
                    if port_names.insert(port.as_str()) {
                        members.clear();
                        members.extend(refs_bits(refs));
                        let mut bits = Vec::with_capacity(members.len());
                        for m in &members {
                            let p = make_port(m, &dirs, &decl_range, &mut bit_dir, pool);
                            bits.extend(p.bits.iter().copied());
                            cell_ports.push(p);
                        }
                        cell_ports.push(CellPort { name: pool.intern(port), bits, bundle: true });
                    }
                }
                HeaderPort::Unnamed { .. } => {}
            }
        }
        // As a TOP (`linkNetwork`): each entry's bits, in header order, that are a port's — once.
        let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
        for h in &self.header {
            let bits = match h {
                // a whole declared name is every bit its declaration has — a `[0:0]` port's one bit
                // is `x[0]` — as `makeCellPort` made it
                HeaderPort::Named { refs, .. } => match decl_range.get(refs) {
                    Some(&(h, l)) if dirs.contains_key(refs) => bus_bits(refs, h, l),
                    _ => expand_bits(refs, &decl_range),
                },
                HeaderPort::Bundle { refs, .. } | HeaderPort::Unnamed { refs } => refs_bits(refs),
            };
            for b in bits {
                let Some(&d) = bit_dir.get(&b) else { continue };
                if !seen.insert(b.clone()) {
                    continue;
                }
                let s = pool.intern(&b);
                match d {
                    Some(PortDir::Input) => nl.inputs.push(s),
                    Some(PortDir::Output) => nl.outputs.push(s),
                    Some(PortDir::Inout) => nl.inouts.push(s),
                    _ => nl.undirected.push(s),
                }
            }
        }
        nl.cell_ports = cell_ports;
        // ---- held connections, expanded to bits now that every declaration is read -----------
        if self.pos_tagged || self.named_deferred {
            let (old_c, old_t) = (std::mem::take(&mut nl.conn_table), std::mem::take(&mut nl.tie_table));
            for x in nl.insts.iter_mut() {
                let (cs, ts) = (&old_c[x.conns.0 as usize..x.conns.1 as usize], &old_t[x.ties.0 as usize..x.ties.1 as usize]);
                // a held named pin, whole: its items in order — a name, or a constant bit
                let mut held: Vec<(String, Vec<(u32, Result<String, bool>)>)> = Vec::new();
                let mut item = |pin: &str, order: u32, v: Result<String, bool>| match held.iter_mut().find(|h| h.0 == pin) {
                    Some(h) => h.1.push((order, v)),
                    None => held.push((pin.to_string(), vec![(order, v)])),
                };
                for &(p, net) in cs {
                    if let Some((order, pin)) = read_named_tag(pool.get(p)) {
                        item(pin, order, Ok(pool.get(net).to_string()));
                    }
                }
                for &(p, v) in ts {
                    if let Some((order, pin)) = read_named_tag(pool.get(p)) {
                        item(pin, order, Err(v));
                    }
                }
                let (c0, t0) = (nl.conn_table.len() as u32, nl.tie_table.len() as u32);
                for &(p, v) in ts {
                    if read_named_tag(pool.get(p)).is_none() {
                        nl.tie_table.push((p, v));
                    }
                }
                let mut emitted = vec![false; held.len()];
                for &(p, net) in cs {
                    if let Some((_, pin)) = read_named_tag(pool.get(p)) {
                        // the whole pin, where its first bit was
                        let h = held.iter().position(|h| h.0 == pin).unwrap_or(0);
                        if std::mem::replace(&mut emitted[h], true) {
                            continue;
                        }
                        let (pin, items) = (held[h].0.clone(), &mut held[h].1);
                        items.sort_by_key(|e| e.0);
                        let bits: Vec<Result<String, bool>> = items
                            .iter()
                            .flat_map(|(_, v)| match v {
                                Ok(n) => expand_bits(n, &decl_range).into_iter().map(Ok).collect::<Vec<_>>(),
                                Err(b) => vec![Err(*b)],
                            })
                            .collect();
                        let k = bits.len();
                        for (idx, bit) in bits.into_iter().enumerate() {
                            let p = pool.intern(&if k == 1 { pin.clone() } else { format!("{pin}[{}]", k - 1 - idx) });
                            match bit {
                                Ok(n) => nl.conn_table.push((p, pool.intern(&n))),
                                Err(v) => nl.tie_table.push((p, v)),
                            }
                        }
                        continue;
                    }
                    let Some((k, b, _)) = read_pos_tag(pool.get(p)) else {
                        nl.conn_table.push((p, net));
                        continue;
                    };
                    let net = pool.get(net).to_string();
                    for (s, bit) in expand_bits(&net, &decl_range).into_iter().enumerate() {
                        let tag = pos_tag(pool, k, b, Some(s as u32));
                        nl.conn_table.push((tag, pool.intern(&bit)));
                    }
                }
                x.conns = (c0, nl.conn_table.len() as u32);
                x.ties = (t0, nl.tie_table.len() as u32);
            }
        }
        // A DUPLICATED instance name is renamed, as the reference's `checkInstanceName` does: the
        // later one becomes `<name>_<k>`, k the first from 1 not already among the names seen so
        // far. Kept, two instances answered to one name.
        let mut seen: std::collections::HashSet<Sym> = std::collections::HashSet::with_capacity(nl.insts.len());
        for x in nl.insts.iter_mut() {
            if !seen.insert(x.name) {
                let base = pool.get(x.name).to_string();
                let mut k = 1;
                let renamed = loop {
                    let s = pool.intern(&format!("{base}_{k}"));
                    if !seen.contains(&s) {
                        break s;
                    }
                    k += 1;
                };
                x.name = renamed;
                seen.insert(renamed);
            }
        }
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
    /// declaration's `= rhs` is one too — but its left side is UNDECLARED, one bit (`isNamed`):
    /// `wire [1:0] v = w;` is 1 bit against 2, and merges nothing.
    #[test]
    fn an_assign_merges_bit_for_bit() {
        let src = "module top (x, y, z);\n  input [0:1] x;\n  output [1:0] y;\n  output [2:0] z;\n  wire [1:0] w;\n  wire [1:0] v = w;\n  wire s = x[0], q;\n  assign w = x, z = {y[0], 2'b10};\n  assign y = {q, s};\n  assign q = z;\nendmodule";
        let nl = parse(src).unwrap();
        let a: Vec<String> = nl.aliases.iter().map(|(l, r)| format!("{l}={r}")).collect();
        assert_eq!(
            a,
            ["s=x[0]", "w[1]=x[0]", "w[0]=x[1]", "z[2]=y[0]", "z[1]=1'b1", "z[0]=1'b0", "y[1]=q", "y[0]=s"]
        );
        assert_eq!(nl.skipped_assigns, 2, "q (1 bit) = z (3 bits), and v (undeclared) = w (2 bits), are not merged");
        assert!(!nl.aliases.iter().any(|(l, _)| l == "q" || l.starts_with("q[")), "q is not declared [1:0]");
    }

    /// Rule (the reference's `makeModule`): a module defined twice keeps its LAST definition; the
    /// earlier one is gone.
    #[test]
    fn a_module_defined_twice_keeps_the_last() {
        let src = "module leaf (input a, output y);\n  INV u0 (.A(a), .Y(y));\nendmodule\nmodule top (input a, output y);\n  leaf l (.a(a), .y(y));\nendmodule\nmodule leaf (input a, output y);\n  BUF u1 (.A(a), .Y(y));\nendmodule\n";
        let d = parse_all(src).unwrap();
        assert_eq!(d.module_names(), ["top", "leaf"]);
        assert_eq!(d.module("leaf").unwrap().insts[0].name, "u1");
        assert_eq!(d.top().module, "top");
    }

    /// Rule (the reference's `makeModuleInstBody`): a `supply0` / `supply1` net is held at 0 / 1
    /// by its declared name, and pins stay connected to it.
    #[test]
    fn a_supply_net_is_constant() {
        let src = "module top (input a, output y);\n  supply0 VSS;\n  supply1 VDD, VPWR;\n  INV u0 (.A(a), .VGND(VSS), .VPWR(VDD), .Y(y));\nendmodule";
        let nl = parse(src).unwrap();
        let c: Vec<(&str, bool)> = nl.constants.iter().map(|(n, v)| (n.as_str(), *v)).collect();
        assert_eq!(c, [("VSS", false), ("VDD", true), ("VPWR", true)]);
        assert!(nl.insts[0].conns.iter().any(|(p, n)| p == "VGND" && n == "VSS"));
    }

    /// A net type after a port direction in the body (`output reg q;`) is not a port name.
    /// (The reference's grammar rejects `output reg` outside the header; this reader accepts it.)
    #[test]
    fn a_net_type_after_a_direction_is_not_a_port() {
        let nl = parse("module top (a, q);\n  input wire a;\n  output reg [1:0] q;\nendmodule").unwrap();
        assert_eq!(nl.inputs, ["a"]);
        assert_eq!(nl.outputs, ["q[1]", "q[0]"]);
    }

    /// Rule (the reference's `makeNamedInstPins`): a bus pin of a module DEFINED in the file takes
    /// the connection's bits in the CHILD's declared order — its port member iterator — paired one
    /// for one with the connection's bits, left index first. `ci[0:3]` on `a[3:0]` is ci[0]=a[3];
    /// `co[3:0]` on `b[0:3]` is co[3]=b[0]. (A cell not defined here keeps the Liberty-bus naming,
    /// first bit on the highest index.)
    #[test]
    fn a_defined_childs_pin_bits_follow_its_declared_order() {
        let src = "module top (a, b);\n input [3:0] a;\n input [0:3] b;\n child u (.ci(a), .co(b));\n CELL c (.D(b));\nendmodule\nmodule child (ci, co);\n input [0:3] ci;\n input [3:0] co;\nendmodule\n";
        let d = parse_all(src).unwrap();
        let top = d.top();
        let conns = |name: &str| -> Vec<(String, String)> { top.insts.iter().find(|x| x.name == name).unwrap().conns.clone() };
        let want = |v: &[(&str, &str)]| -> Vec<(String, String)> { v.iter().map(|(p, n)| (p.to_string(), n.to_string())).collect() };
        assert_eq!(
            conns("u"),
            want(&[("ci[0]", "a[3]"), ("ci[1]", "a[2]"), ("ci[2]", "a[1]"), ("ci[3]", "a[0]"), ("co[3]", "b[0]"), ("co[2]", "b[1]"), ("co[1]", "b[2]"), ("co[0]", "b[3]")])
        );
        assert_eq!(conns("c"), want(&[("D[3]", "b[0]"), ("D[2]", "b[1]"), ("D[1]", "b[2]"), ("D[0]", "b[3]")]));
    }

    /// Rule (the reference's `makeNamedInstPins`): a named pin of a module DEFINED in the file is
    /// left unconnected when the module has no such port (warning 201) or the port's width is not
    /// the connection's (warning 200) — counted in `unbound_pins`. A black box is not checked.
    #[test]
    fn a_defined_childs_pin_of_the_wrong_width_or_no_port_is_unconnected() {
        let src = "module top (a, s);\n input [3:0] a;\n input s;\n child u (.i(a), .k(s), .z(s), .w(a[1:0]));\n CELL c (.i(a), .z(s));\nendmodule\nmodule child (i, k, w);\n input [1:0] i;\n input k;\n input [1:0] w;\nendmodule\n";
        let d = parse_all(src).unwrap();
        let top = d.top();
        let pins = |n: &str| -> Vec<String> { top.insts.iter().find(|x| x.name == n).unwrap().conns.iter().map(|(p, _)| p.clone()).collect() };
        // i: 4 bits on a 2-bit port; z: no such port — both dropped. k and w bind.
        assert_eq!(pins("u"), ["k", "w[1]", "w[0]"]);
        assert_eq!(pins("c"), ["i[3]", "i[2]", "i[1]", "i[0]", "z"]);
        assert_eq!(top.unbound_pins, 5);
    }

    /// Rule (the reference lexer): an escaped identifier is a NAME, whatever it spells — `\output `
    /// is a port called `output`, `\; ` and `\endmodule ` are nets, `\1x ` an instance — never the
    /// keyword or punctuation its text reads as.
    #[test]
    fn an_escaped_keyword_or_symbol_is_a_name() {
        let src = "module subk (input a, output \\output );\n  wire \\; ;\n  INV_X1 g1 (.A(a), .ZN(\\; ));\n  INV_X1 \\1x  (.A(\\; ), .ZN(\\output ));\n  INV_X1 g3 (.A(a), .ZN(\\endmodule ));\nendmodule\nmodule top (input a, output z);\n  subk u1 (.a(a), .\\output (z));\nendmodule\n";
        let d = parse_all(src).unwrap();
        let sub = d.module("subk").unwrap();
        assert_eq!(sub.outputs, ["output"]);
        let insts: Vec<(&str, Vec<(String, String)>)> = sub.insts.iter().map(|x| (x.name.as_str(), x.conns.clone())).collect();
        let c = |v: &[(&str, &str)]| -> Vec<(String, String)> { v.iter().map(|(p, n)| (p.to_string(), n.to_string())).collect() };
        assert_eq!(insts, [("g1", c(&[("A", "a"), ("ZN", ";")])), ("1x", c(&[("A", ";"), ("ZN", "output")])), ("g3", c(&[("A", "a"), ("ZN", "endmodule")]))]);
        assert_eq!(d.top().insts[0].conns, c(&[("a", "a"), ("output", "z")]));
        assert!(d.escaped_names().contains("output"));
    }

    /// Rule (the reference's `makeCellPorts` and `linkNetwork`): the HEADER says what is a port,
    /// in its order. A declared `input` it does not list is a net (warning 197); `{a, b}` makes no
    /// port (2721); `y[1]` is a port NAMED `y[1]`, with no declaration and so no direction (166);
    /// `i[1:0]` is the port `i` — every bit its declaration has as a child, bits 1..0 as a top.
    #[test]
    fn the_header_says_what_is_a_port() {
        let d = parse_all("module sub (z, {a, b}, i[1:0], w, y[1]);\n input a, b, w, k;\n input [3:0] i;\n output z;\n output [1:0] y;\nendmodule\n").unwrap();
        let sub = d.module("sub").unwrap();
        assert_eq!(sub.inputs, ["i[1]", "i[0]", "w"], "a, b and k are nets; i shows the bits the header names");
        assert_eq!(sub.outputs, ["z"]);
        assert_eq!(sub.undirected, ["y[1]"]);
        // header order, not declaration order
        let nl = parse("module m (y, a);\n input a;\n output y;\nendmodule\n").unwrap();
        assert_eq!((nl.inputs.as_slice(), nl.outputs.as_slice()), (&["a".to_string()][..], &["y".to_string()][..]));
        // as a CHILD, `i` is all four bits: a two-bit connection does not fit it
        let d = parse_all("module top (p, q);\n input [1:0] p;\n output q;\n sub u (.i(p), .z(q));\nendmodule\nmodule sub (i[1:0], z);\n input [3:0] i;\n output z;\nendmodule\n").unwrap();
        assert_eq!(d.top().insts[0].conns, [("z".to_string(), "q".to_string())]);
        assert_eq!(d.top().unbound_pins, 2);
        // a one-bit vector port is its one bit, `x[0]`; an escaped scalar `\\wi[0] ` is a port
        // a parent connects by that whole name
        let d = parse_all("module top (x, \\t[0] );\n input [0:0] x;\n input \\t[0] ;\n leaf u (.\\wi[0] (\\t[0] ), .v(x[0]));\nendmodule\nmodule leaf (\\wi[0] , v);\n input \\wi[0] ;\n input [0:0] v;\nendmodule\n").unwrap();
        assert_eq!(d.top().inputs, ["x[0]", "t[0]"]);
        assert_eq!(d.top().insts[0].conns, [("wi[0]".to_string(), "t[0]".to_string()), ("v[0]".to_string(), "x[0]".to_string())]);
    }

    /// Rule (the reference's `makeOrderedInstPins`): a module defined in the file binds POSITIONAL
    /// values to its ports in header order, bit by bit from each port's left index; a value of the
    /// wrong width (warning 202), one on a bundle port (2723) and one past the last port connect
    /// nothing. Any other cell keeps the empty pin name, a bit per pin.
    #[test]
    fn positional_values_bind_to_a_defined_childs_ports_in_header_order() {
        let src = "module top (a, c, y, z);\n input [3:0] a;\n input c;\n output [1:0] y;\n output z;\n \
                   sub u1 (y, {c, 1'b0}, a);\n sub u2 (z, a, a, c);\n bun u3 (c, z, a[1:0]);\n LEAF g (a[1:0], z);\nendmodule\n\
                   module sub (o, i, d);\n output [0:1] o;\n input [1:0] i;\n input [3:0] d;\nendmodule\n\
                   module bun (.p({e, f}), g);\n input e, f, g;\nendmodule\n";
        let d = parse_all(src).unwrap();
        let top = d.top();
        let inst = |n: &str| top.insts.iter().find(|x| x.name == n).unwrap();
        let s = |v: &[(&str, &str)]| -> Vec<(String, String)> { v.iter().map(|(p, n)| (p.to_string(), n.to_string())).collect() };
        assert_eq!(inst("u1").conns, s(&[("o[0]", "y[1]"), ("o[1]", "y[0]"), ("i[1]", "c"), ("d[3]", "a[3]"), ("d[2]", "a[2]"), ("d[1]", "a[1]"), ("d[0]", "a[0]")]));
        assert_eq!(inst("u1").ties, [("i[0]".to_string(), false)]);
        // z is one bit on a two-bit port, a four on a two, then a on d, and c is past the end
        assert_eq!(inst("u2").conns, s(&[("d[3]", "a[3]"), ("d[2]", "a[2]"), ("d[1]", "a[1]"), ("d[0]", "a[0]")]));
        // bun's ports are e, f, then the bundle p, then g: c goes to e, z to f, and a[1:0] — the
        // bundle's width — still connects nothing
        assert_eq!(inst("u3").conns, s(&[("e", "c"), ("f", "z")]));
        assert_eq!(inst("g").conns, s(&[("", "a[1]"), ("", "a[0]"), ("", "z")]));
        assert_eq!(top.unbound_pins, 1 + 4 + 1 + 2);
        assert_eq!(top.positional, 3);
    }

    /// Rule (the reference's link): a name's width is its DECLARATION's wherever that is — a
    /// declaration after the instance that connects it counts, in the connection's own place.
    #[test]
    fn a_declaration_after_the_instance_still_gives_the_width() {
        let src = "module top (a, y);\n CELL u (.Q(y), .D({1'b1, a}), .E(a[0]));\n input [1:0] a;\n output y;\nendmodule\n";
        let nl = parse(src).unwrap();
        let c: Vec<String> = nl.insts[0].conns.iter().map(|(p, n)| format!("{p}={n}")).collect();
        assert_eq!(c, ["Q=y", "D[1]=a[1]", "D[0]=a[0]", "E=a[0]"]);
        assert_eq!(nl.insts[0].ties, [("D[2]".to_string(), true)]);
        // and a defined child binds it by its port's width
        let d = parse_all("module top (a, y);\n sub u (.i(a), .o(y));\n input [3:0] a;\n output [3:0] y;\nendmodule\nmodule sub (i, o);\n input [3:0] i;\n output [3:0] o;\nendmodule\n").unwrap();
        assert_eq!(d.top().insts[0].conns.len(), 8);
        assert_eq!(d.top().unbound_pins, 0);
    }

    /// Rule (the reference lexer, flex longest match): an escaped identifier runs to the next
    /// whitespace, so `//` and `/*` inside it are part of the name — a comment only starts where a
    /// token could. Outside one, both comment forms still remove what they cover.
    #[test]
    fn a_comment_marker_inside_an_escaped_name_is_part_of_it() {
        let src = "module top (input a, output z);\n  wire \\a//b ;\n  INV_X1 u1 (.A(a), .ZN(\\a//b )); // a real comment (.A(z))\n  INV_X1 \\u/*2  (.A(\\a//b ), .ZN(z)); /* BUF_X1 x (.A(a)); */\nendmodule\n";
        let nl = parse(src).unwrap();
        let insts: Vec<(&str, Vec<(String, String)>)> = nl.insts.iter().map(|x| (x.name.as_str(), x.conns.clone())).collect();
        let c = |v: &[(&str, &str)]| -> Vec<(String, String)> { v.iter().map(|(p, n)| (p.to_string(), n.to_string())).collect() };
        assert_eq!(insts, [("u1", c(&[("A", "a"), ("ZN", "a//b")])), ("u/*2", c(&[("A", "a//b"), ("ZN", "z")]))]);
    }

    /// An escaped VECTOR whose name holds brackets (`input [0:3] \ti[0] ;`) connected whole is
    /// that vector, bit by bit — not a bit-select of a bus `ti`.
    #[test]
    fn an_escaped_vector_named_like_a_select_expands_whole() {
        let src = "module top (\\ti[0] );\n input [0:3] \\ti[0] ;\n child u (.\\wi[0] (\\ti[0] ));\nendmodule\nmodule child (\\wi[0] );\n input [0:3] \\wi[0] ;\nendmodule\n";
        let d = parse_all(src).unwrap();
        let got: Vec<(String, String)> = d.top().insts[0].conns.clone();
        let want: Vec<(String, String)> = (0..4).map(|k| (format!("wi[0][{k}]"), format!("ti[0][{k}]"))).collect();
        assert_eq!(got, want);
        // and a part-select of it is a range of THAT vector
        let src = "module top (\\ti[0] );\n input [0:3] \\ti[0] ;\n CELL c (.D(\\ti[0] [1:2]));\nendmodule\n";
        let nl = parse(src).unwrap();
        assert_eq!(nl.insts[0].conns, [("D[1]".to_string(), "ti[0][1]".to_string()), ("D[0]".to_string(), "ti[0][2]".to_string())]);
    }

    /// An escaped scalar port `\foo[3] ` is canonically `foo[3]`, the name of a bit of the bus
    /// `foo` beside it: it is not a ninth bit, and the bus's bits are not renumbered around it.
    #[test]
    fn an_escaped_scalar_is_not_a_bit_of_the_bus_it_names() {
        let src = "module top (foo, \\foo[3] , y);\n input [7:0] foo;\n input \\foo[3] ;\n output y;\n child u (.foo(foo), .\\foo[3] (\\foo[3] ), .y(y));\nendmodule\nmodule child (foo, \\foo[3] , y);\n input [7:0] foo;\n input \\foo[3] ;\n output y;\nendmodule\n";
        let d = parse_all(src).unwrap();
        let u = &d.top().insts[0];
        for k in 0..8 {
            let p = format!("foo[{k}]");
            assert!(u.conns.iter().any(|(a, n)| a == &p && n == &p), "{p} is on {p}");
        }
    }

    /// Rule (the reference's `checkInstanceName`): a later instance with a name already used is
    /// renamed `<name>_<k>`, k the first from 1 not among the names seen so far.
    #[test]
    fn a_duplicated_instance_name_is_renamed() {
        let src = "module top (input a, output y);\n  INV u (.A(a), .Y(y));\n  INV u_1 (.A(a), .Y(y));\n  INV u (.A(a), .Y(y));\n  INV u (.A(a), .Y(y));\nendmodule";
        let nl = parse(src).unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["u", "u_1", "u_2", "u_3"]);
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
