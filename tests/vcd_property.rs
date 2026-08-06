//! Property test: VCDs generated in-process, with toggle counts known by construction.
//!
//! `tools/gen_vcd.py` does the same thing on disk, because the FST and SAIF cross-checks need
//! real files for GTKWave and Verilator to convert. This one needs nothing at all — no Python,
//! no PDK, no external tool — so it runs on every `cargo test`, which is where a reader that
//! decides how much power a design burns ought to be checked.
//!
//! The generator decides every value change itself, so the expected count is arithmetic. It
//! deliberately produces the cases that cost us four defects: `x`/`z` excursions, `$dumpoff`
//! windows, vectors whose leading zeros are dropped, and one identifier code shared by two
//! names.

use std::collections::BTreeMap;

use vyges_loom::vcd::Vcd;

/// Deterministic PRNG — no dev-dependency, and a failing seed is reproducible from its number
/// alone, which is the only property that matters for a property test.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn chance(&mut self, pct: u64) -> bool {
        self.next() % 100 < pct
    }
}

/// One generated dump plus the counts it must produce.
struct Case {
    text: String,
    expect: BTreeMap<String, u64>,
}

fn generate(seed: u64) -> Case {
    let mut r = Rng(seed | 1);
    let nscal = 1 + r.below(4);
    let nvec = r.below(3);
    let vw = 2 + r.below(5); // includes widths that are not a multiple of 8

    let mut decls = String::new();
    let mut code = 33u8; // '!'
    let mut scal = Vec::new();
    for i in 0..nscal {
        let c = code as char;
        code += 1;
        decls.push_str(&format!("$var wire 1 {c} s{i} $end\n"));
        scal.push((format!("s{i}"), c));
    }
    // one alias: a second name sharing the first signal's code
    let aliased = nscal > 0 && r.chance(30);
    if aliased {
        decls.push_str(&format!("$var wire 1 {} s0_alias $end\n", scal[0].1));
    }
    let mut vecs = Vec::new();
    for i in 0..nvec {
        let c = code as char;
        code += 1;
        decls.push_str(&format!("$var wire {vw} {c} v{i} [{}:0] $end\n", vw - 1));
        vecs.push((format!("v{i}"), c));
    }

    let mut expect: BTreeMap<String, u64> = BTreeMap::new();
    let mut last: BTreeMap<String, char> = BTreeMap::new();
    let mut vlast: BTreeMap<String, Vec<char>> = BTreeMap::new();
    for (n, _) in &scal {
        expect.insert(format!("top.{n}"), 0);
    }
    if aliased {
        expect.insert("top.s0_alias".into(), 0);
    }
    for (n, _) in &vecs {
        for b in 0..vw {
            expect.insert(format!("top.{n}[{b}]"), 0);
        }
    }

    let mut body = String::from("#0\n$dumpvars\n");
    for (n, c) in &scal {
        let v = if r.chance(50) { '0' } else { '1' };
        body.push_str(&format!("{v}{c}\n"));
        last.insert(n.clone(), v);
    }
    for (n, c) in &vecs {
        let bits: Vec<char> = (0..vw).map(|_| if r.chance(50) { '0' } else { '1' }).collect();
        body.push_str(&format!("b{} {c}\n", bits.iter().collect::<String>()));
        vlast.insert(n.clone(), bits);
    }
    body.push_str("$end\n");

    let mut t = 0u64;
    let mut dumping = true;
    for _ in 0..(5 + r.below(30)) {
        t += 1 + r.below(10) as u64;
        body.push_str(&format!("#{t}\n"));
        if r.chance(10) {
            // a $dumpoff x-flood and its $dumpon restore are not activity
            if dumping {
                body.push_str("$dumpoff\n");
                for (_, c) in &scal {
                    body.push_str(&format!("x{c}\n"));
                }
                for (_, c) in &vecs {
                    body.push_str(&format!("bx {c}\n"));
                }
                body.push_str("$end\n");
                dumping = false;
            } else {
                body.push_str("$dumpon\n");
                for (n, c) in &scal {
                    body.push_str(&format!("{}{c}\n", last[n]));
                }
                for (n, c) in &vecs {
                    body.push_str(&format!("b{} {c}\n", vlast[n].iter().collect::<String>()));
                }
                body.push_str("$end\n");
                dumping = true;
            }
            continue;
        }
        if !dumping {
            continue;
        }
        for (n, c) in &scal {
            if !r.chance(50) {
                continue;
            }
            let v = match r.below(6) {
                0 => 'x',
                1 => 'z',
                k => {
                    if k % 2 == 0 {
                        '0'
                    } else {
                        '1'
                    }
                }
            };
            if v == '0' || v == '1' {
                if last.get(n).is_some_and(|p| *p != v) {
                    *expect.get_mut(&format!("top.{n}")).unwrap() += 1;
                    if aliased && n == "s0" {
                        *expect.get_mut("top.s0_alias").unwrap() += 1;
                    }
                }
                last.insert(n.clone(), v);
            }
            body.push_str(&format!("{v}{c}\n"));
        }
        for (n, c) in &vecs {
            if !r.chance(50) {
                continue;
            }
            let bits: Vec<char> = (0..vw)
                .map(|_| match r.below(10) {
                    0 => 'x',
                    1 => 'z',
                    k => {
                        if k % 2 == 0 {
                            '0'
                        } else {
                            '1'
                        }
                    }
                })
                .collect();
            for (i, b) in bits.iter().enumerate() {
                if *b != '0' && *b != '1' {
                    continue;
                }
                let prev = vlast[n][i];
                if prev == '0' || prev == '1' {
                    if prev != *b {
                        // bit i of a [vw-1:0] vector is named [vw-1-i]
                        *expect.get_mut(&format!("top.{n}[{}]", vw - 1 - i)).unwrap() += 1;
                    }
                }
                vlast.get_mut(n).unwrap()[i] = *b;
            }
            // VCD drops leading ZEROS, but a reader extends with the leading character when it
            // is x or z — so a leading 0 must be kept when the next character is not 0 or 1.
            let mut sv: String = bits.iter().collect();
            while sv.len() > 1 && sv.starts_with('0') && matches!(sv.as_bytes()[1], b'0' | b'1') {
                sv.remove(0);
            }
            body.push_str(&format!("b{sv} {c}\n"));
        }
    }
    t += 1 + r.below(10) as u64;
    body.push_str(&format!("#{t}\n"));

    Case {
        text: format!(
            "$timescale 1ns $end\n$scope module top $end\n{decls}$upscope $end\n\
             $enddefinitions $end\n{body}"
        ),
        expect,
    }
}

/// **The property**: for any generated dump, the reader reproduces the constructed counts
/// exactly — no more, no fewer, and no nets it was never given.
#[test]
fn generated_dumps_are_counted_exactly() {
    let mut failures = Vec::new();
    for seed in 1..=500u64 {
        let case = generate(seed);
        let v = match Vcd::parse(&case.text) {
            Ok(v) => v,
            Err(e) => {
                failures.push(format!("seed {seed}: {e}"));
                continue;
            }
        };
        for (net, want) in &case.expect {
            let got = v.idx.toggles.get(net).copied().unwrap_or(0);
            if got != *want {
                failures.push(format!("seed {seed}: {net} want {want} got {got}"));
            }
        }
        for net in v.idx.toggles.keys() {
            if !case.expect.contains_key(net) {
                failures.push(format!("seed {seed}: reader invented {net}"));
            }
        }
        if failures.len() > 8 {
            break;
        }
    }
    assert!(
        failures.is_empty(),
        "{} failure(s) — rerun one with `generate(<seed>)`:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// The same property under windowing: the halves must partition the whole.
#[test]
fn generated_dumps_partition_across_windows() {
    let mut failures = Vec::new();
    for seed in 1..=200u64 {
        let case = generate(seed);
        let Ok(whole) = Vcd::parse(&case.text) else { continue };
        if whole.sim_time_s <= 0.0 {
            continue;
        }
        let mid = whole.sim_time_s / 2.0;
        let (Ok(a), Ok(b)) = (
            Vcd::parse_windowed(&case.text, Some((0.0, Some(mid)))),
            Vcd::parse_windowed(&case.text, Some((mid, None))),
        ) else {
            continue;
        };
        for (path, n) in &whole.idx.toggles {
            let split = a.idx.toggles.get(path).copied().unwrap_or(0)
                + b.idx.toggles.get(path).copied().unwrap_or(0);
            if *n != split {
                failures.push(format!("seed {seed}: {path} whole={n} halves={split}"));
            }
        }
        if failures.len() > 8 {
            break;
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
