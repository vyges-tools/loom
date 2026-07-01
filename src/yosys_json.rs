//! Yosys JSON netlist reader (`write_json` output) → the shared [`netlist::Netlist`].
//!
//! Yosys is the synthesizer in the open flow; its `write_json` output is a common,
//! unambiguous netlist interchange (no Verilog-parsing ambiguity). This reader maps
//! it onto the same [`netlist::Netlist`] model the structural-Verilog reader produces,
//! so either front-end feeds one design.
//!
//! Shape consumed:
//! ```json
//! { "modules": { "top": {
//!     "attributes": { "top": "00000000000000000000000000000001" },
//!     "ports":    { "a": {"direction":"input","bits":[2]} },
//!     "cells":    { "g1": {"type":"INV","connections":{"A":[2],"Y":[3]}} },
//!     "netnames": { "n1": {"bits":[3]} }
//! } } }
//! ```
//! Nets are integer bit-ids; the constants `"0"`/`"1"`/`"x"`/`"z"` are dropped (as the
//! structural reader drops `1'b0`). Multi-bit nets/ports expand to `name[i]` so bits
//! line up with the rest of the toolchain. Pure std — a small JSON parser is bundled
//! (no serde). Top module: the one with a truthy `top` attribute, else the sole
//! module, else an error listing the candidates (loom is explicit — no silent pick).

use crate::netlist::{Inst, Netlist, NetlistError};

pub fn load(path: &str) -> Result<Netlist, NetlistError> {
    let text = std::fs::read_to_string(path).map_err(|e| NetlistError(format!("{path}: {e}")))?;
    parse(&text)
}

pub fn parse(text: &str) -> Result<Netlist, NetlistError> {
    let v = json::parse(text).map_err(|e| NetlistError(format!("JSON: {e}")))?;
    let modules = v.get("modules").ok_or_else(|| NetlistError("not a Yosys netlist (no \"modules\")".into()))?;
    let mods = modules.as_object().ok_or_else(|| NetlistError("\"modules\" is not an object".into()))?;
    if mods.is_empty() {
        return Err(NetlistError("no modules in netlist".into()));
    }

    // Pick the top module.
    let top_idx = mods
        .iter()
        .position(|(_, m)| m.get("attributes").and_then(|a| a.get("top")).is_some_and(attr_truthy));
    let (name, module) = match top_idx {
        Some(i) => (&mods[i].0, &mods[i].1),
        None if mods.len() == 1 => (&mods[0].0, &mods[0].1),
        None => {
            let names: Vec<&str> = mods.iter().map(|(n, _)| n.as_str()).collect();
            return Err(NetlistError(format!(
                "{} modules and no `top` attribute — cannot choose: {}",
                mods.len(),
                names.join(", ")
            )));
        }
    };

    let mut nl = Netlist { module: strip_esc(name), ..Netlist::default() };

    // bit-id → net name, from netnames (so cell connections resolve to real names).
    let mut bit_name: Vec<(i64, String)> = Vec::new();
    if let Some(netnames) = module.get("netnames").and_then(|n| n.as_object()) {
        for (nname, obj) in netnames {
            let bits = obj.get("bits").and_then(|b| b.as_array()).map(|a| a.as_slice()).unwrap_or(&[]);
            for (idx, bit) in bits.iter().enumerate() {
                if let Some(id) = bit.as_int() {
                    bit_name.push((id, expand(nname, idx, bits.len())));
                }
            }
        }
    }
    let name_of = |id: i64| bit_name.iter().find(|(b, _)| *b == id).map(|(_, n)| n.clone());

    // Ports → inputs / outputs (expanded per bit).
    if let Some(ports) = module.get("ports").and_then(|p| p.as_object()) {
        for (pname, obj) in ports {
            let dir = obj.get("direction").and_then(|d| d.as_str()).unwrap_or("");
            let bits = obj.get("bits").and_then(|b| b.as_array()).map(|a| a.as_slice()).unwrap_or(&[]);
            for (idx, bit) in bits.iter().enumerate() {
                let Some(id) = bit.as_int() else { continue };
                let net = name_of(id).unwrap_or_else(|| expand(pname, idx, bits.len()));
                match dir {
                    "input" => nl.inputs.push(net),
                    "output" => nl.outputs.push(net),
                    _ => {} // inout / unknown: skipped in v0
                }
            }
        }
    }

    // Cells → instances.
    if let Some(cells) = module.get("cells").and_then(|c| c.as_object()) {
        for (cname, obj) in cells {
            let cell_type = obj.get("type").and_then(|t| t.as_str()).unwrap_or("").to_string();
            let mut inst = Inst { cell: strip_esc(&cell_type), name: strip_esc(cname), conns: Vec::new() };
            if let Some(conns) = obj.get("connections").and_then(|c| c.as_object()) {
                for (pin, bits_v) in conns {
                    let bits = bits_v.as_array().map(|a| a.as_slice()).unwrap_or(&[]);
                    for (idx, bit) in bits.iter().enumerate() {
                        let pin_name = expand(pin, idx, bits.len());
                        match bit {
                            // constant bits ("0"/"1"/"x"/"z") are dropped
                            json::Value::Str(_) => {}
                            _ => {
                                if let Some(id) = bit.as_int() {
                                    let net = name_of(id).unwrap_or_else(|| format!("$n{id}"));
                                    inst.conns.push((pin_name, net));
                                }
                            }
                        }
                    }
                }
            }
            nl.insts.push(inst);
        }
    }

    Ok(nl)
}

/// `name` for a scalar, `name[i]` for a bit of a multi-bit net/port.
fn expand(name: &str, idx: usize, width: usize) -> String {
    let n = strip_esc(name);
    if width > 1 {
        format!("{n}[{idx}]")
    } else {
        n
    }
}

/// Yosys escapes public names with a leading `\`; drop it.
fn strip_esc(s: &str) -> String {
    s.strip_prefix('\\').unwrap_or(s).to_string()
}

/// A Yosys attribute value is a bit-string (or int/str). Truthy = contains a set bit
/// or a non-zero number.
fn attr_truthy(v: &json::Value) -> bool {
    match v {
        json::Value::Str(s) => s.chars().any(|c| c == '1'),
        json::Value::Num(n) => *n != 0.0,
        json::Value::Bool(b) => *b,
        _ => false,
    }
}

// ---- minimal std-only JSON parser --------------------------------------------

mod json {
    /// A JSON value. Objects keep insertion order (deterministic iteration).
    #[derive(Debug, Clone)]
    pub enum Value {
        Null,
        Bool(bool),
        Num(f64),
        Str(String),
        Arr(Vec<Value>),
        Obj(Vec<(String, Value)>),
    }

    impl Value {
        pub fn get(&self, key: &str) -> Option<&Value> {
            match self {
                Value::Obj(m) => m.iter().find(|(k, _)| k == key).map(|(_, v)| v),
                _ => None,
            }
        }
        pub fn as_object(&self) -> Option<&Vec<(String, Value)>> {
            match self {
                Value::Obj(m) => Some(m),
                _ => None,
            }
        }
        pub fn as_array(&self) -> Option<&Vec<Value>> {
            match self {
                Value::Arr(a) => Some(a),
                _ => None,
            }
        }
        pub fn as_str(&self) -> Option<&str> {
            match self {
                Value::Str(s) => Some(s),
                _ => None,
            }
        }
        /// An integer net-id: a JSON number (Yosys emits bit-ids as numbers).
        pub fn as_int(&self) -> Option<i64> {
            match self {
                Value::Num(n) if n.fract() == 0.0 => Some(*n as i64),
                _ => None,
            }
        }
    }

    pub fn parse(text: &str) -> Result<Value, String> {
        let b = text.as_bytes();
        let mut p = Parser { b, i: 0 };
        p.ws();
        let v = p.value()?;
        p.ws();
        if p.i != b.len() {
            return Err(format!("trailing data at byte {}", p.i));
        }
        Ok(v)
    }

    struct Parser<'a> {
        b: &'a [u8],
        i: usize,
    }

    impl Parser<'_> {
        fn ws(&mut self) {
            while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\t' | b'\n' | b'\r') {
                self.i += 1;
            }
        }

        fn value(&mut self) -> Result<Value, String> {
            self.ws();
            match self.b.get(self.i) {
                Some(b'{') => self.object(),
                Some(b'[') => self.array(),
                Some(b'"') => Ok(Value::Str(self.string()?)),
                Some(b't') => self.lit("true", Value::Bool(true)),
                Some(b'f') => self.lit("false", Value::Bool(false)),
                Some(b'n') => self.lit("null", Value::Null),
                Some(_) => self.number(),
                None => Err("unexpected end of JSON".into()),
            }
        }

        fn lit(&mut self, word: &str, v: Value) -> Result<Value, String> {
            if self.b[self.i..].starts_with(word.as_bytes()) {
                self.i += word.len();
                Ok(v)
            } else {
                Err(format!("invalid literal at byte {}", self.i))
            }
        }

        fn object(&mut self) -> Result<Value, String> {
            self.i += 1; // {
            let mut m = Vec::new();
            self.ws();
            if self.b.get(self.i) == Some(&b'}') {
                self.i += 1;
                return Ok(Value::Obj(m));
            }
            loop {
                self.ws();
                if self.b.get(self.i) != Some(&b'"') {
                    return Err(format!("expected string key at byte {}", self.i));
                }
                let key = self.string()?;
                self.ws();
                if self.b.get(self.i) != Some(&b':') {
                    return Err(format!("expected ':' at byte {}", self.i));
                }
                self.i += 1;
                let val = self.value()?;
                m.push((key, val));
                self.ws();
                match self.b.get(self.i) {
                    Some(b',') => self.i += 1,
                    Some(b'}') => {
                        self.i += 1;
                        return Ok(Value::Obj(m));
                    }
                    _ => return Err(format!("expected ',' or '}}' at byte {}", self.i)),
                }
            }
        }

        fn array(&mut self) -> Result<Value, String> {
            self.i += 1; // [
            let mut a = Vec::new();
            self.ws();
            if self.b.get(self.i) == Some(&b']') {
                self.i += 1;
                return Ok(Value::Arr(a));
            }
            loop {
                a.push(self.value()?);
                self.ws();
                match self.b.get(self.i) {
                    Some(b',') => self.i += 1,
                    Some(b']') => {
                        self.i += 1;
                        return Ok(Value::Arr(a));
                    }
                    _ => return Err(format!("expected ',' or ']' at byte {}", self.i)),
                }
            }
        }

        fn string(&mut self) -> Result<String, String> {
            self.i += 1; // opening quote
            let mut s = String::new();
            while let Some(&c) = self.b.get(self.i) {
                self.i += 1;
                match c {
                    b'"' => return Ok(s),
                    b'\\' => {
                        let e = *self.b.get(self.i).ok_or("bad escape")?;
                        self.i += 1;
                        match e {
                            b'"' => s.push('"'),
                            b'\\' => s.push('\\'),
                            b'/' => s.push('/'),
                            b'n' => s.push('\n'),
                            b't' => s.push('\t'),
                            b'r' => s.push('\r'),
                            b'b' => s.push('\u{08}'),
                            b'f' => s.push('\u{0c}'),
                            b'u' => {
                                let hex = self.b.get(self.i..self.i + 4).ok_or("bad \\u escape")?;
                                let code = u32::from_str_radix(std::str::from_utf8(hex).map_err(|_| "bad \\u")?, 16)
                                    .map_err(|_| "bad \\u hex")?;
                                self.i += 4;
                                s.push(char::from_u32(code).unwrap_or('\u{fffd}'));
                            }
                            other => return Err(format!("bad escape '\\{}'", other as char)),
                        }
                    }
                    _ => {
                        // pass through the UTF-8 byte
                        s.push(c as char);
                        // handle multi-byte UTF-8 by copying continuation bytes verbatim
                        if c >= 0x80 {
                            s.pop();
                            let start = self.i - 1;
                            while self.b.get(self.i).is_some_and(|b| b & 0xC0 == 0x80) {
                                self.i += 1;
                            }
                            s.push_str(std::str::from_utf8(&self.b[start..self.i]).map_err(|_| "bad utf8")?);
                        }
                    }
                }
            }
            Err("unterminated string".into())
        }

        fn number(&mut self) -> Result<Value, String> {
            let start = self.i;
            while let Some(&c) = self.b.get(self.i) {
                if matches!(c, b'0'..=b'9' | b'-' | b'+' | b'.' | b'e' | b'E') {
                    self.i += 1;
                } else {
                    break;
                }
            }
            let s = std::str::from_utf8(&self.b[start..self.i]).map_err(|_| "bad number")?;
            s.parse::<f64>().map(Value::Num).map_err(|_| format!("invalid number {s:?}"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHAIN: &str = r#"{
      "modules": {
        "top": {
          "attributes": { "top": "00000000000000000000000000000001" },
          "ports": {
            "a": { "direction": "input",  "bits": [2] },
            "y": { "direction": "output", "bits": [5] }
          },
          "cells": {
            "g1": { "type": "INV", "connections": { "A": [2], "Y": [3] } },
            "g2": { "type": "INV", "connections": { "A": [3], "Y": [4] } },
            "g3": { "type": "INV", "connections": { "A": [4], "Y": [5] } }
          },
          "netnames": {
            "a":  { "bits": [2] },
            "n1": { "bits": [3] },
            "n2": { "bits": [4] },
            "y":  { "bits": [5] }
          }
        }
      }
    }"#;

    #[test]
    fn reads_inverter_chain() {
        let nl = parse(CHAIN).unwrap();
        assert_eq!(nl.module, "top");
        assert_eq!(nl.inputs, ["a"]);
        assert_eq!(nl.outputs, ["y"]);
        assert_eq!(nl.insts.len(), 3);
        let g2 = nl.insts.iter().find(|i| i.name == "g2").unwrap();
        assert_eq!(g2.cell, "INV");
        // connections resolve bit-ids back to net names
        assert!(g2.conns.contains(&("A".into(), "n1".into())));
        assert!(g2.conns.contains(&("Y".into(), "n2".into())));
    }

    #[test]
    fn constant_bits_are_dropped() {
        let j = r#"{"modules":{"m":{"cells":{"u":{"type":"BUF",
            "connections":{"A":["0"],"Y":[3]}}},"netnames":{"o":{"bits":[3]}}}}}"#;
        let nl = parse(j).unwrap();
        let u = &nl.insts[0];
        assert_eq!(u.conns, [("Y".to_string(), "o".to_string())]); // A=const dropped
    }

    #[test]
    fn multibit_expands() {
        let j = r#"{"modules":{"m":{"ports":{"d":{"direction":"input","bits":[2,3]}},
            "netnames":{"d":{"bits":[2,3]}}}}}"#;
        let nl = parse(j).unwrap();
        assert_eq!(nl.inputs, ["d[0]", "d[1]"]);
    }

    #[test]
    fn multiple_modules_without_top_errors() {
        let j = r#"{"modules":{"a":{},"b":{}}}"#;
        let err = parse(j).unwrap_err();
        assert!(err.to_string().contains("cannot choose"));
    }

    #[test]
    fn not_a_netlist_errors() {
        assert!(parse(r#"{"foo":1}"#).unwrap_err().to_string().contains("modules"));
    }
}
