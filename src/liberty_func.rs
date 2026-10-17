// SPDX-License-Identifier: Apache-2.0
//! Liberty `function` / `three_state` expressions as the reference's expression parser builds
//! them, and the timing sense of a pin through one (what a timing group with no `timing_sense`
//! gets).
//!
//! The grammar:
//! - `terminal`: a port, `0`, `1`, or `( expr )`;
//! - `terminal_expr`: a terminal, `!` terminal, or terminal `'` — negation applies to a TERMINAL;
//! - `implicit_and`: two or more terminal_exprs side by side, folded LEFT into ANDs. It binds
//!   tighter than every operator (`a ^ b c` is `a ^ (b c)`): the parser shifts there;
//! - `expr`: a terminal_expr, an implicit_and, or `expr op expr` with every operator left
//!   associative and, from loosest to tightest, `+` `|` (or), `*` `&` (and), `^` (xor).
//!
//! A port the cell does not declare makes the whole expression absent (every operator over an
//! absent operand is absent), which [`Func::parse_in`] reports as `None`.

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Func {
    Port(String),
    Not(Box<Func>),
    Or(Box<Func>, Box<Func>),
    And(Box<Func>, Box<Func>),
    Xor(Box<Func>, Box<Func>),
    One,
    Zero,
}

/// A pin's timing sense through an expression.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Sense {
    Positive,
    Negative,
    NonUnate,
    None,
    Unknown,
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Port(String),
    Op(char),
}

/// Single-character operators and parentheses; everything else up to one of them or a blank is a
/// port name (`0` and `1` alone are constants).
fn scan(s: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let flush = |cur: &mut String, out: &mut Vec<Tok>| {
        if !cur.is_empty() {
            out.push(Tok::Port(std::mem::take(cur)));
        }
    };
    for c in s.chars() {
        match c {
            '!' | '\'' | '+' | '|' | '*' | '&' | '^' | '(' | ')' => {
                flush(&mut cur, &mut out);
                out.push(Tok::Op(c));
            }
            '"' | ';' => flush(&mut cur, &mut out),
            c if c.is_whitespace() => flush(&mut cur, &mut out),
            c => cur.push(c),
        }
    }
    flush(&mut cur, &mut out);
    out
}

/// Binding power of a binary operator (higher binds tighter), all left associative.
fn bp(c: char) -> Option<u8> {
    match c {
        '+' | '|' => Some(1),
        '*' | '&' => Some(2),
        '^' => Some(3),
        _ => None,
    }
}

struct Parser {
    toks: Vec<Tok>,
    at: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.at)
    }

    fn terminal(&mut self) -> Option<Func> {
        match self.toks.get(self.at).cloned() {
            Some(Tok::Port(p)) => {
                self.at += 1;
                Some(match p.as_str() {
                    "0" => Func::Zero,
                    "1" => Func::One,
                    _ => Func::Port(p),
                })
            }
            Some(Tok::Op('(')) => {
                self.at += 1;
                let e = self.expr(0)?;
                if self.peek() != Some(&Tok::Op(')')) {
                    return None;
                }
                self.at += 1;
                Some(e)
            }
            _ => None,
        }
    }

    fn starts_terminal_expr(&self) -> bool {
        matches!(self.peek(), Some(Tok::Port(_)) | Some(Tok::Op('(')) | Some(Tok::Op('!')))
    }

    fn terminal_expr(&mut self) -> Option<Func> {
        if self.peek() == Some(&Tok::Op('!')) {
            self.at += 1;
            return Some(Func::Not(Box::new(self.terminal()?)));
        }
        let t = self.terminal()?;
        if self.peek() == Some(&Tok::Op('\'')) {
            self.at += 1;
            return Some(Func::Not(Box::new(t)));
        }
        Some(t)
    }

    /// A terminal_expr followed by any juxtaposed ones, folded left into ANDs.
    fn primary(&mut self) -> Option<Func> {
        let mut e = self.terminal_expr()?;
        while self.starts_terminal_expr() {
            let r = self.terminal_expr()?;
            e = Func::And(Box::new(e), Box::new(r));
        }
        Some(e)
    }

    fn expr(&mut self, min_bp: u8) -> Option<Func> {
        let mut lhs = self.primary()?;
        while let Some(Tok::Op(c)) = self.peek().cloned() {
            let Some(p) = bp(c) else { break };
            if p <= min_bp {
                break;
            }
            self.at += 1;
            let rhs = self.expr(p)?;
            lhs = match c {
                '+' | '|' => Func::Or(Box::new(lhs), Box::new(rhs)),
                '*' | '&' => Func::And(Box::new(lhs), Box::new(rhs)),
                _ => Func::Xor(Box::new(lhs), Box::new(rhs)),
            };
        }
        Some(lhs)
    }
}

impl Func {
    /// Parse `s`; `None` on a syntax error or when it names a port `is_port` rejects.
    pub(crate) fn parse_in(s: &str, is_port: impl Fn(&str) -> bool) -> Option<Func> {
        let mut p = Parser { toks: scan(s), at: 0 };
        let e = p.expr(0)?;
        if p.at != p.toks.len() || !e.ports_all(&is_port) {
            return None;
        }
        Some(e)
    }

    fn ports_all(&self, ok: &impl Fn(&str) -> bool) -> bool {
        match self {
            Func::Port(p) => ok(p),
            Func::Not(a) => a.ports_all(ok),
            Func::Or(a, b) | Func::And(a, b) | Func::Xor(a, b) => a.ports_all(ok) && b.ports_all(ok),
            Func::One | Func::Zero => true,
        }
    }

    /// Does the expression name `port`?
    pub(crate) fn has_port(&self, port: &str) -> bool {
        match self {
            Func::Port(p) => p == port,
            Func::Not(a) => a.has_port(port),
            Func::Or(a, b) | Func::And(a, b) | Func::Xor(a, b) => a.has_port(port) || b.has_port(port),
            Func::One | Func::Zero => false,
        }
    }

    /// The reference's `invert`: a negation is unwrapped, anything else negated (how a
    /// `three_state` DISABLE expression becomes the enable).
    pub(crate) fn invert(self) -> Func {
        match self {
            Func::Not(a) => *a,
            e => Func::Not(Box::new(e)),
        }
    }

    /// The timing sense of `port` through the expression, by the reference's rules.
    pub(crate) fn port_timing_sense(&self, port: &str) -> Sense {
        use Sense::*;
        match self {
            Func::Port(p) => {
                if p == port {
                    Positive
                } else {
                    None
                }
            }
            Func::Not(a) => match a.port_timing_sense(port) {
                Positive => Negative,
                Negative => Positive,
                s => s,
            },
            Func::Or(a, b) | Func::And(a, b) => {
                let (l, r) = (a.port_timing_sense(port), b.port_timing_sense(port));
                if l == r {
                    l
                } else if l == NonUnate || r == NonUnate || (l == Positive && r == Negative) || (l == Negative && r == Positive) {
                    NonUnate
                } else if l == None || l == Unknown {
                    r
                } else if r == None || r == Unknown {
                    l
                } else {
                    Unknown
                }
            }
            Func::Xor(a, b) => {
                let (l, r) = (a.port_timing_sense(port), b.port_timing_sense(port));
                if [l, r].iter().any(|s| matches!(s, Positive | Negative | NonUnate)) {
                    NonUnate
                } else {
                    Unknown
                }
            }
            Func::One | Func::Zero => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sense(f: &str, p: &str) -> Sense {
        Func::parse_in(f, |_| true).unwrap().port_timing_sense(p)
    }

    /// Juxtaposition binds tightest; negation is of a terminal; operators left associative.
    #[test]
    fn the_grammar_groups_as_the_reference_parser_does() {
        let p = |s: &str| Func::parse_in(s, |_| true).unwrap();
        let port = |s: &str| Box::new(Func::Port(s.into()));
        assert_eq!(p("A ^ B C"), Func::Xor(port("A"), Box::new(Func::And(port("B"), port("C")))));
        assert_eq!(p("A B + C"), Func::Or(Box::new(Func::And(port("A"), port("B"))), port("C")));
        assert_eq!(p("!A&B"), Func::And(Box::new(Func::Not(port("A"))), port("B")));
        assert_eq!(p("(A+B)'"), Func::Not(Box::new(Func::Or(port("A"), port("B")))));
        assert_eq!(p("\"A|B\""), Func::Or(port("A"), port("B")));
    }

    /// The reference's `portTimingSense`, case by case.
    #[test]
    fn port_timing_sense_follows_the_reference() {
        assert_eq!(sense("A", "A"), Sense::Positive);
        assert_eq!(sense("!A", "A"), Sense::Negative);
        assert_eq!(sense("!(A&B)", "B"), Sense::Negative);
        assert_eq!(sense("(A&B)|(!A&C)", "A"), Sense::NonUnate); // a mux select
        assert_eq!(sense("(A&B)|(!A&C)", "C"), Sense::Positive); // none | positive -> positive
        assert_eq!(sense("A^B", "A"), Sense::NonUnate);
        assert_eq!(sense("B^C", "A"), Sense::Unknown); // xor over two `none`
        assert_eq!(sense("1", "A"), Sense::None);
        assert_eq!(sense("B", "A"), Sense::None);
    }

    /// A port the cell lacks makes the expression absent; so does a syntax error.
    #[test]
    fn an_unknown_port_makes_no_expression() {
        assert!(Func::parse_in("A&IQ", |p| p == "A").is_none());
        assert!(Func::parse_in("A&(B", |_| true).is_none());
        assert_eq!(Func::parse_in("!EN", |_| true).unwrap().invert(), Func::Port("EN".into()));
    }
}
