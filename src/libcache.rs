//! Cross-process on-disk parse cache for [`crate::liberty::Lib`] (#38).
//!
//! Persists parsed Libs under `~/.vyges/cache/liberty/` so separate processes
//! (e.g. a `vyges-sta-si` run and a `vyges-power` run) that load the same library
//! skip re-parsing it. Sits behind `Lib::load_opts`, *below* the in-process cache
//! (#37): in-process → on-disk → parse.
//!
//! **Off by default** — enable with `VYGES_LIB_CACHE=1`. Shipping the mechanism this
//! way can never regress anyone; flip it on once the box benchmark confirms decode is
//! meaningfully faster than a Liberty re-parse (the issue's ship gate).
//!
//! Format: a hand-rolled little-endian binary codec (loom is std-only — no serde),
//! magic-tagged and versioned so a format change invalidates rather than misreads.
//! Content-addressed filenames `{hash}-{len}-{ccs}.vlc` (the same key as the
//! in-process cache) mean a changed library is a different entry — never a stale read.

use crate::ccs::{CcsArc, CcsWaveform};
use crate::liberty::{Arc, Cell, Constraint, Dir, Lib, Pin, RecvCap, Table};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

const MAGIC: &[u8; 4] = b"VLC1"; // Vyges Liberty Cache — bump the trailing digit on any format change

// ── byte writer / reader (little-endian, std-only) ───────────────────────────────

#[derive(Default)]
struct W {
    b: Vec<u8>,
}
impl W {
    fn u8(&mut self, v: u8) {
        self.b.push(v);
    }
    fn u64(&mut self, v: u64) {
        self.b.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.b.extend_from_slice(&v.to_le_bytes());
    }
    fn s(&mut self, v: &str) {
        self.u64(v.len() as u64);
        self.b.extend_from_slice(v.as_bytes());
    }
    fn vf64(&mut self, v: &[f64]) {
        self.u64(v.len() as u64);
        for &x in v {
            self.f64(x);
        }
    }
}

struct R<'a> {
    b: &'a [u8],
    i: usize,
}
impl R<'_> {
    fn u8(&mut self) -> Option<u8> {
        let v = *self.b.get(self.i)?;
        self.i += 1;
        Some(v)
    }
    fn u64(&mut self) -> Option<u64> {
        let end = self.i.checked_add(8)?;
        let s = self.b.get(self.i..end)?;
        self.i = end;
        Some(u64::from_le_bytes(s.try_into().ok()?))
    }
    fn f64(&mut self) -> Option<f64> {
        let end = self.i.checked_add(8)?;
        let s = self.b.get(self.i..end)?;
        self.i = end;
        Some(f64::from_le_bytes(s.try_into().ok()?))
    }
    fn s(&mut self) -> Option<String> {
        let n = self.u64()? as usize;
        let end = self.i.checked_add(n)?;
        let s = self.b.get(self.i..end)?;
        self.i = end;
        Some(std::str::from_utf8(s).ok()?.to_string())
    }
    fn vf64(&mut self) -> Option<Vec<f64>> {
        let n = self.u64()? as usize;
        let mut v = Vec::with_capacity(n.min(1 << 20));
        for _ in 0..n {
            v.push(self.f64()?);
        }
        Some(v)
    }
}

// ── per-struct encode / decode ───────────────────────────────────────────────────

fn enc_table(w: &mut W, t: &Table) {
    w.vf64(&t.index_1);
    w.vf64(&t.index_2);
    w.u64(t.values.len() as u64);
    for row in &t.values {
        w.vf64(row);
    }
}
fn dec_table(r: &mut R) -> Option<Table> {
    let index_1 = r.vf64()?;
    let index_2 = r.vf64()?;
    let n = r.u64()? as usize;
    let mut values = Vec::with_capacity(n.min(1 << 16));
    for _ in 0..n {
        values.push(r.vf64()?);
    }
    Some(Table { index_1, index_2, values })
}

fn enc_wave(w: &mut W, x: &CcsWaveform) {
    w.f64(x.in_slew);
    w.f64(x.out_cap);
    w.f64(x.ref_time);
    w.vf64(&x.time);
    w.vf64(&x.current);
}
fn dec_wave(r: &mut R) -> Option<CcsWaveform> {
    Some(CcsWaveform {
        in_slew: r.f64()?,
        out_cap: r.f64()?,
        ref_time: r.f64()?,
        time: r.vf64()?,
        current: r.vf64()?,
    })
}

fn enc_ccs(w: &mut W, c: &CcsArc) {
    w.u64(c.rise.len() as u64);
    for x in &c.rise {
        enc_wave(w, x);
    }
    w.u64(c.fall.len() as u64);
    for x in &c.fall {
        enc_wave(w, x);
    }
}
fn dec_ccs(r: &mut R) -> Option<CcsArc> {
    let nr = r.u64()? as usize;
    let mut rise = Vec::with_capacity(nr.min(1 << 16));
    for _ in 0..nr {
        rise.push(dec_wave(r)?);
    }
    let nf = r.u64()? as usize;
    let mut fall = Vec::with_capacity(nf.min(1 << 16));
    for _ in 0..nf {
        fall.push(dec_wave(r)?);
    }
    Some(CcsArc { rise, fall })
}

fn enc_constraint(w: &mut W, c: &Constraint) {
    enc_table(w, &c.rise);
    enc_table(w, &c.fall);
}
fn dec_constraint(r: &mut R) -> Option<Constraint> {
    Some(Constraint { rise: dec_table(r)?, fall: dec_table(r)? })
}

fn enc_recv(w: &mut W, rc: &RecvCap) {
    enc_table(w, &rc.c1_rise);
    enc_table(w, &rc.c2_rise);
    enc_table(w, &rc.c1_fall);
    enc_table(w, &rc.c2_fall);
}
fn dec_recv(r: &mut R) -> Option<RecvCap> {
    Some(RecvCap {
        c1_rise: dec_table(r)?,
        c2_rise: dec_table(r)?,
        c1_fall: dec_table(r)?,
        c2_fall: dec_table(r)?,
    })
}

fn enc_arc(w: &mut W, a: &Arc) {
    w.s(&a.related_pin);
    w.s(&a.sense);
    enc_table(w, &a.cell_rise);
    enc_table(w, &a.cell_fall);
    enc_table(w, &a.rise_transition);
    enc_table(w, &a.fall_transition);
    enc_ccs(w, &a.ccs);
    enc_table(w, &a.sigma_rise);
    enc_table(w, &a.sigma_fall);
}
fn dec_arc(r: &mut R) -> Option<Arc> {
    Some(Arc {
        related_pin: r.s()?,
        sense: r.s()?,
        cell_rise: dec_table(r)?,
        cell_fall: dec_table(r)?,
        rise_transition: dec_table(r)?,
        fall_transition: dec_table(r)?,
        ccs: dec_ccs(r)?,
        sigma_rise: dec_table(r)?,
        sigma_fall: dec_table(r)?,
    })
}

fn dir_u8(d: Dir) -> u8 {
    match d {
        Dir::In => 0,
        Dir::Out => 1,
        Dir::Inout => 2,
        Dir::Other => 3,
    }
}
fn u8_dir(v: u8) -> Option<Dir> {
    Some(match v {
        0 => Dir::In,
        1 => Dir::Out,
        2 => Dir::Inout,
        3 => Dir::Other,
        _ => return None,
    })
}

fn enc_pin(w: &mut W, p: &Pin) {
    w.s(&p.name);
    w.u8(dir_u8(p.direction));
    w.f64(p.capacitance);
    w.f64(p.cap_f);
    match &p.recv {
        Some(rc) => {
            w.u8(1);
            enc_recv(w, rc);
        }
        None => w.u8(0),
    }
    w.u8(p.clock as u8);
    w.u64(p.setup.len() as u64);
    for c in &p.setup {
        enc_constraint(w, c);
    }
    w.u64(p.hold.len() as u64);
    for c in &p.hold {
        enc_constraint(w, c);
    }
    w.u64(p.arcs.len() as u64);
    for a in &p.arcs {
        enc_arc(w, a);
    }
}
fn dec_pin(r: &mut R) -> Option<Pin> {
    let name = r.s()?;
    let direction = u8_dir(r.u8()?)?;
    let capacitance = r.f64()?;
    let cap_f = r.f64()?;
    let recv = match r.u8()? {
        0 => None,
        1 => Some(dec_recv(r)?),
        _ => return None,
    };
    let clock = r.u8()? != 0;
    let ns = r.u64()? as usize;
    let mut setup = Vec::with_capacity(ns.min(1 << 12));
    for _ in 0..ns {
        setup.push(dec_constraint(r)?);
    }
    let nh = r.u64()? as usize;
    let mut hold = Vec::with_capacity(nh.min(1 << 12));
    for _ in 0..nh {
        hold.push(dec_constraint(r)?);
    }
    let na = r.u64()? as usize;
    let mut arcs = Vec::with_capacity(na.min(1 << 12));
    for _ in 0..na {
        arcs.push(dec_arc(r)?);
    }
    Some(Pin { name, direction, capacitance, cap_f, recv, clock, setup, hold, arcs })
}

fn enc_cell(w: &mut W, c: &Cell) {
    w.s(&c.name);
    w.u64(c.pins.len() as u64);
    for (k, p) in &c.pins {
        w.s(k);
        enc_pin(w, p);
    }
    w.u8(c.is_seq as u8);
    match &c.clock_pin {
        Some(s) => {
            w.u8(1);
            w.s(s);
        }
        None => w.u8(0),
    }
    w.f64(c.leakage_w);
    w.f64(c.int_energy_j);
}
fn dec_cell(r: &mut R) -> Option<Cell> {
    let name = r.s()?;
    let np = r.u64()? as usize;
    let mut pins = BTreeMap::new();
    for _ in 0..np {
        let k = r.s()?;
        pins.insert(k, dec_pin(r)?);
    }
    let is_seq = r.u8()? != 0;
    let clock_pin = match r.u8()? {
        0 => None,
        1 => Some(r.s()?),
        _ => return None,
    };
    let leakage_w = r.f64()?;
    let int_energy_j = r.f64()?;
    Some(Cell { name, pins, is_seq, clock_pin, leakage_w, int_energy_j })
}

/// Serialize a `Lib` to the cache byte format (magic + version + payload).
pub fn encode(lib: &Lib) -> Vec<u8> {
    let mut w = W::default();
    w.b.extend_from_slice(MAGIC);
    w.f64(lib.voltage);
    w.u64(lib.cells.len() as u64);
    for (k, c) in &lib.cells {
        w.s(k);
        enc_cell(&mut w, c);
    }
    w.b
}

/// Deserialize a `Lib` from cache bytes. Returns `None` on any mismatch or malformed
/// input — the caller then treats it as a cache miss and re-parses (never a stale read).
pub fn decode(bytes: &[u8]) -> Option<Lib> {
    if bytes.len() < 4 || &bytes[0..4] != MAGIC {
        return None;
    }
    let mut r = R { b: bytes, i: 4 };
    let voltage = r.f64()?;
    let n = r.u64()? as usize;
    let mut cells = BTreeMap::new();
    for _ in 0..n {
        let k = r.s()?;
        cells.insert(k, dec_cell(&mut r)?);
    }
    Some(Lib { cells, voltage })
}

// ── on-disk cache ────────────────────────────────────────────────────────────────

/// Resolve the cache directory, or `None` when the cache is disabled (default) or no
/// HOME. Enable with `VYGES_LIB_CACHE` set to anything non-empty.
fn cache_dir() -> Option<PathBuf> {
    match std::env::var_os("VYGES_LIB_CACHE") {
        Some(v) if !v.is_empty() => {}
        _ => return None,
    }
    let home = std::env::var_os("HOME")?;
    let dir = Path::new(&home).join(".vyges").join("cache").join("liberty");
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir)
}

fn entry_path(dir: &Path, key: (u64, u64, bool)) -> PathBuf {
    dir.join(format!("{:016x}-{}-{}.vlc", key.0, key.1, key.2 as u8))
}

/// Look up a cached `Lib` for `key` in `dir`. `None` = miss (or corrupt/old-format).
pub fn disk_get_in(dir: &Path, key: (u64, u64, bool)) -> Option<Lib> {
    let bytes = std::fs::read(entry_path(dir, key)).ok()?;
    decode(&bytes)
}

/// Persist `lib` under `key` in `dir` (write-to-temp + atomic rename). Best-effort.
pub fn disk_put_in(dir: &Path, key: (u64, u64, bool), lib: &Lib) {
    let bytes = encode(lib);
    let tmp = dir.join(format!(".tmp-{}-{:016x}", std::process::id(), key.0));
    if std::fs::write(&tmp, &bytes).is_ok() && std::fs::rename(&tmp, entry_path(dir, key)).is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
}

/// Env-gated disk lookup (no-op unless `VYGES_LIB_CACHE` is set).
pub fn disk_get(key: (u64, u64, bool)) -> Option<Lib> {
    disk_get_in(&cache_dir()?, key)
}

/// Env-gated disk store (no-op unless `VYGES_LIB_CACHE` is set).
pub fn disk_put(key: (u64, u64, bool), lib: &Lib) {
    if let Some(dir) = cache_dir() {
        disk_put_in(&dir, key, lib);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::LibOpts;

    const LIB: &str = r#"
library (demo) {
  capacitive_load_unit (1, pf);
  cell (INV) {
    pin (A) {
      direction : input; capacitance : 0.004;
      receiver_capacitance () { receiver_capacitance1_rise (t) { values("0.001, 0.002"); } }
    }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t) { values("0.1, 0.2"); }
        output_current_rise () {
          vector (v) { index_1("0.01"); index_2("0.005"); index_3("0.0, 0.1"); values("0.0, 0.5"); }
        }
      }
    }
  }
}
"#;

    #[test]
    fn codec_round_trips_byte_stable() {
        let lib = Lib::parse(LIB).unwrap();
        let bytes = encode(&lib);
        let back = decode(&bytes).expect("decode");
        // encode∘decode is the identity on bytes → lossless round-trip.
        assert_eq!(bytes, encode(&back));
        assert_eq!(back.cells.len(), lib.cells.len());
        assert!((back.voltage - lib.voltage).abs() < 1e-12);
    }

    #[test]
    fn decode_rejects_bad_magic_and_truncation() {
        assert!(decode(b"").is_none());
        assert!(decode(b"XXXX").is_none());
        let mut bytes = encode(&Lib::parse(LIB).unwrap());
        bytes.truncate(bytes.len() - 4); // chop a float → malformed
        assert!(decode(&bytes).is_none());
    }

    // Generate a large synthetic NLDM library: `cells` cells, each with a 7×7-table
    // delay arc — the shape and float-parsing cost of a real corner lib.
    fn big_lib(cells: usize) -> String {
        let idx = "0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64";
        let table = |name: &str, base: f64| {
            let mut t = format!("        {name} (t) {{ index_1(\"{idx}\"); index_2(\"{idx}\");\n          values(");
            let rows: Vec<String> = (0..7)
                .map(|i| {
                    let r: Vec<String> =
                        (0..7).map(|j| format!("{:.5}", base + i as f64 * 0.011 + j as f64 * 0.003)).collect();
                    format!("\"{}\"", r.join(", "))
                })
                .collect();
            t.push_str(&rows.join(", "));
            t.push_str("); }\n");
            t
        };
        let mut s = String::from("library (big) {\n  capacitive_load_unit (1, pf);\n");
        for k in 0..cells {
            s.push_str(&format!("  cell (CELL{k}) {{\n    cell_leakage_power : 1.0;\n"));
            s.push_str("    pin (A) { direction : input; capacitance : 0.002; }\n");
            s.push_str("    pin (Y) { direction : output;\n      timing () { related_pin : \"A\";\n");
            s.push_str(&table("cell_rise", 0.10));
            s.push_str(&table("cell_fall", 0.12));
            s.push_str(&table("rise_transition", 0.03));
            s.push_str(&table("fall_transition", 0.04));
            s.push_str("      }\n    }\n  }\n");
        }
        s.push_str("}\n");
        s
    }

    #[test]
    #[ignore = "benchmark; run with: cargo test --release --lib bench_decode_vs_parse -- --ignored --nocapture"]
    fn bench_decode_vs_parse() {
        use std::time::Instant;
        let text = big_lib(2000);
        let bytes = encode(&Lib::parse(&text).unwrap());
        println!("\nlib text {} KB → cache {} KB", text.len() / 1024, bytes.len() / 1024);
        let reps = 5;
        let mut t_parse = std::time::Duration::MAX;
        let mut t_decode = std::time::Duration::MAX;
        for _ in 0..reps {
            let a = Instant::now();
            let l = Lib::parse(&text).unwrap();
            t_parse = t_parse.min(a.elapsed());
            std::hint::black_box(l);
            let b = Instant::now();
            let l2 = decode(&bytes).unwrap();
            t_decode = t_decode.min(b.elapsed());
            std::hint::black_box(l2);
        }
        let sp = t_parse.as_secs_f64() * 1e3;
        let sd = t_decode.as_secs_f64() * 1e3;
        println!("parse: {sp:.2} ms   decode: {sd:.2} ms   speedup: {:.1}×\n", sp / sd);
        assert!(sd < sp, "decode should beat re-parse");
    }

    #[test]
    fn disk_put_get_round_trips_in_dir() {
        let lib = Lib::parse(LIB).unwrap();
        let key = (0xabc_1234u64, LIB.len() as u64, false);
        let dir = std::env::temp_dir().join(format!("vyges_libcache_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        assert!(disk_get_in(&dir, key).is_none()); // cold miss
        disk_put_in(&dir, key, &lib);
        let got = disk_get_in(&dir, key).expect("hit after put");
        assert_eq!(got.cells.len(), lib.cells.len());
        // key includes skip_ccs → the NLDM variant is a different entry
        let nldm_key = (key.0, key.1, true);
        assert!(disk_get_in(&dir, nldm_key).is_none());
        disk_put_in(&dir, nldm_key, &Lib::parse_opts(LIB, LibOpts { skip_ccs: true }).unwrap());
        assert!(disk_get_in(&dir, nldm_key).is_some());

        std::fs::remove_dir_all(&dir).ok();
    }
}
