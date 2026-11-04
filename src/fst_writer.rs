//! Clean-room **FST writer**, and the conversions built on it: VCD → FST, FST → FST (a time
//! window and a signal subset), and a bounded FST → VCD export.
//!
//! The files it writes are read the same way by GTKWave's `fst2vcd`, by readers built on the
//! fstcpp library (CoralNPU's `fst_query`), and by [`crate::fst_wave`]. The layout is the one
//! described there; this module writes value-change blocks with the compact alias encoding
//! (type 8), zlib-packed chains, a geometry block, a blackout block when `$dumpoff`/`$dumpon`
//! were used, and a gzip hierarchy.
//!
//! ⛔ **The VCD export is bounded on purpose.** A simulation's FST expands many times over as
//! VCD text; exporting a whole dump can fill a disk, and reading it exhausts an agent's context.
//! [`fst_to_vcd`] therefore requires a time window AND a signal list. To look at a waveform,
//! query the FST instead (`vyges loom fst <file>`).
//!
//! ## How values are recorded (what a reader relies on)
//!
//! - Values set before the first time are the INITIAL state: they go into the first block's
//!   frame, and the dump's start time becomes 0.
//! - A change to the value a signal already holds is dropped, except at time 0, where an
//!   all-`x` value is kept (so a signal explicitly dumped as unknown at 0 is seen).
//! - Two changes of one signal at one time keep the last.
//! - A block is closed when its buffered changes pass the block size; the next starts at the
//!   last time written, with every signal's current value as its frame.

#![deny(clippy::unwrap_used)]
#![cfg_attr(test, allow(clippy::unwrap_used))]

use std::collections::HashMap;
use std::io::{BufRead, Seek, SeekFrom, Write};

use crate::fst::FstError;
use crate::fst_wave::{
    block, fmt_g16, st, tag, vd, vt, FstReader, Hier, Kind, Value, ENDIAN_TEST, HDR_DATE_LEN,
    HDR_PAYLOAD_LEN, HDR_VERSION_LEN, LEVELS,
};

/// Options for a new file.
#[derive(Debug, Clone)]
pub struct WriterOptions {
    /// Power of ten of the time unit (-9 = 1 ns).
    pub timescale: i8,
    /// The writer string the header carries.
    pub writer: String,
    /// The date the header carries; `None` = now (UTC), or `SOURCE_DATE_EPOCH` when set.
    pub date: Option<String>,
    /// 0 Verilog, 1 VHDL, 2 mixed.
    pub file_type: u8,
    pub timezero: i64,
    /// Buffered change bytes that close a block.
    pub block_bytes: usize,
}

impl Default for WriterOptions {
    fn default() -> Self {
        WriterOptions {
            timescale: -9,
            writer: format!("vyges-loom {}", crate::VERSION),
            date: None,
            file_type: 0,
            timezero: 0,
            block_bytes: 64 << 20,
        }
    }
}

fn err<T>(m: impl Into<String>) -> Result<T, FstError> {
    Err(FstError(m.into()))
}

fn io(e: std::io::Error) -> FstError {
    FstError(format!("io: {e}"))
}

pub fn put_uvarint(o: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            o.push(b);
            return;
        }
        o.push(b | 0x80);
    }
}

pub fn put_svarint(o: &mut Vec<u8>, mut v: i64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        let done = (v == 0 && b & 0x40 == 0) || (v == -1 && b & 0x40 != 0);
        if done {
            o.push(b);
            return;
        }
        o.push(b | 0x80);
    }
}

fn zlib(d: &[u8], level: u8) -> Vec<u8> {
    miniz_oxide::deflate::compress_to_vec_zlib(d, level)
}

fn crc32(d: &[u8]) -> u32 {
    let mut table = [0u32; 256];
    for (i, t) in table.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
        *t = c;
    }
    let mut c = !0u32;
    for &b in d {
        c = table[((c ^ b as u32) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

/// A gzip member (RFC 1952) around raw deflate.
pub fn gzip(d: &[u8], level: u8) -> Vec<u8> {
    let mut o = vec![0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 0, 3];
    o.extend(miniz_oxide::deflate::compress_to_vec(d, level));
    o.extend(crc32(d).to_le_bytes());
    o.extend((d.len() as u32).to_le_bytes());
    o
}

/// `asctime` of a UNIX time, as a C library prints it (UTC).
pub fn asctime(secs: i64) -> String {
    const DAYS: [&str; 7] = ["Thu", "Fri", "Sat", "Sun", "Mon", "Tue", "Wed"];
    const MONTHS: [&str; 12] =
        ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    let days = secs.div_euclid(86400);
    let rem = secs.rem_euclid(86400);
    let (h, m, s) = (rem / 3600, rem % 3600 / 60, rem % 60);
    // Civil date from days since 1970-01-01 (Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(mo <= 2);
    format!(
        "{} {} {:2} {:02}:{:02}:{:02} {}\n",
        DAYS[days.rem_euclid(7) as usize],
        MONTHS[(mo - 1) as usize],
        d,
        h,
        m,
        s,
        y
    )
}

fn now_date() -> String {
    let secs = std::env::var("SOURCE_DATE_EPOCH")
        .ok()
        .and_then(|s| s.trim().parse::<i64>().ok())
        .unwrap_or_else(|| {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs() as i64)
                .unwrap_or(0)
        });
    asctime(secs)
}

/// Fit a bit value to `w` bits the way VCD does: a short value extends left with its leading
/// `x`/`z`, otherwise with `0`; a long one keeps its low bits.
pub fn fit_bits(v: &[u8], w: usize) -> Vec<u8> {
    if v.len() == w {
        return v.to_vec();
    }
    if v.len() > w {
        return v[v.len() - w..].to_vec();
    }
    let pad = match v.first().map(|c| c.to_ascii_lowercase()) {
        Some(b'x') => b'x',
        Some(b'z') => b'z',
        _ => b'0',
    };
    let mut o = vec![pad; w - v.len()];
    o.extend_from_slice(v);
    o
}

/// One handle's state in the open block.
#[derive(Default)]
struct Track {
    /// (time index, value bytes) in time order.
    entries: Vec<(u32, Vec<u8>)>,
}

/// A file being written.
pub struct FstWriter<W: Write + Seek> {
    out: W,
    opts: WriterOptions,
    hier: Vec<u8>,
    num_scopes: u64,
    num_vars: u64,
    kinds: Vec<Kind>,
    offsets: Vec<usize>,
    curval: Vec<u8>,
    tracks: Vec<Track>,
    started: bool,
    vc_emitted: bool,
    firsttime: u64,
    curtime: u64,
    block_start: u64,
    block_frame: Vec<u8>,
    times: Vec<u64>,
    buffered: usize,
    sections: u64,
    blackouts: Vec<(u64, bool)>,
}

impl<W: Write + Seek> FstWriter<W> {
    pub fn new(mut out: W, opts: WriterOptions) -> Result<Self, FstError> {
        // The header goes first as a placeholder; `finish` writes it again with the counts.
        out.write_all(&[block::HDR]).map_err(io)?;
        out.write_all(&(HDR_PAYLOAD_LEN as u64 + 8).to_be_bytes()).map_err(io)?;
        out.write_all(&vec![0u8; HDR_PAYLOAD_LEN]).map_err(io)?;
        Ok(FstWriter {
            out,
            opts,
            hier: Vec::new(),
            num_scopes: 0,
            num_vars: 0,
            kinds: Vec::new(),
            offsets: Vec::new(),
            curval: Vec::new(),
            tracks: Vec::new(),
            started: false,
            vc_emitted: false,
            firsttime: 0,
            curtime: 0,
            block_start: 0,
            block_frame: Vec::new(),
            times: Vec::new(),
            buffered: 0,
            sections: 0,
            blackouts: Vec::new(),
        })
    }

    pub fn scope(&mut self, kind: u8, name: &str, component: &str) {
        self.hier.push(tag::SCOPE);
        self.hier.push(if kind as usize >= st::NAMES.len() { 0 } else { kind });
        self.hier.extend_from_slice(name.as_bytes());
        self.hier.push(0);
        self.hier.extend_from_slice(component.as_bytes());
        self.hier.push(0);
        self.num_scopes += 1;
    }

    pub fn upscope(&mut self) {
        self.hier.push(tag::UPSCOPE);
    }

    pub fn attr_begin(&mut self, kind: u8, sub: u8, name: &str, arg: u64) {
        self.hier.push(tag::ATTR_BEGIN);
        self.hier.push(kind);
        self.hier.push(sub);
        self.hier.extend_from_slice(name.as_bytes());
        self.hier.push(0);
        put_uvarint(&mut self.hier, arg);
    }

    pub fn attr_end(&mut self) {
        self.hier.push(tag::ATTR_END);
    }

    /// Declare a variable; `alias` = the handle it shares. Returns its handle (1-based).
    pub fn var(&mut self, kind: u8, dir: u8, name: &str, len: u32, alias: Option<u32>) -> u32 {
        let kind = if kind > vt::MAX { vt::WIRE } else { kind };
        let real = vt::is_real(kind);
        let len = if real {
            8
        } else if kind == vt::STRING {
            0
        } else {
            len
        };
        self.hier.push(kind);
        self.hier.push(if dir as usize >= vd::NAMES.len() { 0 } else { dir });
        self.hier.extend_from_slice(name.as_bytes());
        self.hier.push(0);
        put_uvarint(&mut self.hier, len as u64);
        let alias = alias.filter(|&a| a >= 1 && a as usize <= self.kinds.len());
        put_uvarint(&mut self.hier, alias.unwrap_or(0) as u64);
        self.num_vars += 1;
        if let Some(a) = alias {
            return a;
        }
        let k = if real {
            Kind::Real
        } else if len == 0 {
            Kind::VarLen
        } else {
            Kind::Bits(len)
        };
        self.offsets.push(self.curval.len());
        match k {
            Kind::Bits(n) => self.curval.extend(std::iter::repeat_n(b'x', n as usize)),
            Kind::Real => self.curval.extend(f64::NAN.to_le_bytes()),
            Kind::VarLen => {}
        }
        self.kinds.push(k);
        self.tracks.push(Track::default());
        self.kinds.len() as u32
    }

    /// Advance time. Times must not decrease; a repeat is ignored.
    pub fn time(&mut self, t: u64) -> Result<(), FstError> {
        if !self.started {
            self.started = true;
            self.firsttime = if self.vc_emitted { 0 } else { t };
            self.block_start = self.firsttime;
            self.block_frame = self.curval.clone();
            // Variable-length values have no initial state.
            for tr in &mut self.tracks {
                tr.entries.clear();
            }
            self.times = vec![t];
            self.curtime = t;
            return Ok(());
        }
        if t < self.curtime {
            return err(format!("time goes backwards: {t} after {}", self.curtime));
        }
        if t == self.curtime {
            return Ok(());
        }
        if self.buffered >= self.opts.block_bytes {
            self.flush()?;
            self.block_start = self.curtime;
            self.block_frame = self.curval.clone();
            self.times = vec![self.curtime];
        }
        self.times.push(t);
        self.curtime = t;
        Ok(())
    }

    /// Record a value. Bits are ASCII levels (fitted to the width as VCD does); a real is a
    /// double; a string is bytes.
    pub fn value(&mut self, h: u32, v: &Value) -> Result<(), FstError> {
        let Some(i) = (h as usize).checked_sub(1).filter(|&i| i < self.kinds.len()) else {
            return err(format!("no handle {h}"));
        };
        self.vc_emitted = true;
        let k = self.kinds[i];
        let bytes: Vec<u8> = match (k, v) {
            (Kind::Bits(n), Value::Bits(b)) => fit_bits(b, n as usize),
            (Kind::Bits(n), Value::Real(d)) => fit_bits(format!("{}", *d as i64).as_bytes(), n as usize),
            (Kind::Bits(n), Value::Str(s)) => fit_bits(s, n as usize),
            (Kind::Real, Value::Real(d)) => d.to_le_bytes().to_vec(),
            (Kind::Real, Value::Bits(b)) | (Kind::Real, Value::Str(b)) => {
                let d: f64 = String::from_utf8_lossy(b).trim().parse().unwrap_or(f64::NAN);
                d.to_le_bytes().to_vec()
            }
            (Kind::VarLen, Value::Str(s)) | (Kind::VarLen, Value::Bits(s)) => s.clone(),
            (Kind::VarLen, Value::Real(d)) => fmt_g16(*d).into_bytes(),
        };
        let o = self.offsets[i];
        if k == Kind::VarLen {
            if self.started {
                let ti = (self.times.len() - 1) as u32;
                self.buffered += bytes.len() + 4;
                self.tracks[i].entries.push((ti, bytes));
            }
            return Ok(());
        }
        let fl = k.frame_len();
        if !self.started {
            self.curval[o..o + fl].copy_from_slice(&bytes);
            return Ok(());
        }
        let ti = (self.times.len() - 1) as u32;
        let tr = &mut self.tracks[i];
        if let Some(last) = tr.entries.last_mut() {
            if last.0 == ti {
                last.1 = bytes.clone();
                self.curval[o..o + fl].copy_from_slice(&bytes);
                return Ok(());
            }
        }
        if self.curval[o..o + fl] == bytes[..] {
            let keep = self.curtime == 0 && k != Kind::Real && bytes.iter().all(|&c| c == b'x');
            if !keep {
                return Ok(());
            }
        }
        self.curval[o..o + fl].copy_from_slice(&bytes);
        self.buffered += bytes.len() + 4;
        tr.entries.push((ti, bytes));
        Ok(())
    }

    /// `$dumpon` (true) / `$dumpoff` (false) at the current time.
    pub fn dump_active(&mut self, on: bool) {
        self.blackouts.push((self.curtime, on));
    }

    fn encode_chain(k: Kind, entries: &[(u32, Vec<u8>)]) -> Vec<u8> {
        let mut o = Vec::new();
        let mut prev = 0u32;
        for (ti, v) in entries {
            let d = (ti - prev) as u64;
            prev = *ti;
            match k {
                Kind::Bits(1) => {
                    let c = v.first().copied().unwrap_or(b'x').to_ascii_lowercase();
                    let code = match c {
                        b'0' | b'1' => (((c & 1) as u64) << 1) | (d << 2),
                        _ => {
                            let lvl = LEVELS.iter().position(|&l| l == c).unwrap_or(6) as u64;
                            1 | (lvl << 1) | (d << 4)
                        }
                    };
                    put_uvarint(&mut o, code);
                }
                Kind::VarLen => {
                    put_uvarint(&mut o, d << 1);
                    put_uvarint(&mut o, v.len() as u64);
                    o.extend_from_slice(v);
                }
                Kind::Bits(_) | Kind::Real => {
                    if v.iter().all(|&c| c == b'0' || c == b'1') {
                        put_uvarint(&mut o, d << 1);
                        let mut acc = vec![0u8; v.len().div_ceil(8)];
                        for (j, &c) in v.iter().enumerate() {
                            acc[j / 8] |= (c & 1) << (7 - (j % 8));
                        }
                        o.extend(acc);
                    } else {
                        put_uvarint(&mut o, (d << 1) | 1);
                        o.extend_from_slice(v);
                    }
                }
            }
        }
        o
    }

    /// Write the open block if anything changed in it.
    fn flush(&mut self) -> Result<(), FstError> {
        if self.tracks.iter().all(|t| t.entries.is_empty()) {
            return Ok(());
        }
        let maxh = self.kinds.len() as u64;
        let mut p: Vec<u8> = Vec::new();
        p.extend(self.block_start.to_be_bytes());
        p.extend(self.curtime.to_be_bytes());
        p.extend(0u64.to_be_bytes()); // reader memory, patched below
        let packed = zlib(&self.block_frame, 4);
        put_uvarint(&mut p, self.block_frame.len() as u64);
        let use_packed = packed.len() < self.block_frame.len();
        put_uvarint(&mut p, if use_packed { packed.len() } else { self.block_frame.len() } as u64);
        put_uvarint(&mut p, maxh);
        p.extend_from_slice(if use_packed { &packed } else { &self.block_frame });
        put_uvarint(&mut p, maxh);
        let vc_start = p.len();
        p.push(b'Z');
        // Chains; a chain identical to an earlier one is stored once and aliased.
        let mut memreq = 0u64;
        let mut pos: Vec<i64> = vec![0; self.kinds.len()]; // >0 offset, <0 -(owner+1)
        let mut seen: HashMap<Vec<u8>, usize> = HashMap::new();
        for (i, (track, kind)) in self.tracks.iter().zip(&self.kinds).enumerate() {
            if track.entries.is_empty() {
                continue;
            }
            let raw = Self::encode_chain(*kind, &track.entries);
            memreq += raw.len() as u64;
            let mut c = Vec::new();
            if raw.len() > 32 {
                let z = zlib(&raw, 4);
                if z.len() < raw.len() {
                    put_uvarint(&mut c, raw.len() as u64);
                    c.extend(z);
                } else {
                    put_uvarint(&mut c, 0);
                    c.extend_from_slice(&raw);
                }
            } else {
                put_uvarint(&mut c, 0);
                c.extend_from_slice(&raw);
            }
            if let Some(&owner) = seen.get(&c) {
                pos[i] = -(owner as i64 + 1);
            } else {
                pos[i] = (p.len() - vc_start) as i64;
                p.extend_from_slice(&c);
                seen.insert(c, i);
            }
        }
        // Position table: runs of empty handles, offset deltas, aliases (repeat = 0).
        let ptab_start = p.len();
        let mut prevpos = 0i64;
        let mut prev_alias = 0i64;
        let mut zeros = 0u64;
        for &v in &pos {
            if v == 0 {
                zeros += 1;
                continue;
            }
            if zeros > 0 {
                put_uvarint(&mut p, zeros << 1);
                zeros = 0;
            }
            if v < 0 {
                if v != prev_alias {
                    prev_alias = v;
                    put_svarint(&mut p, (v << 1) | 1);
                } else {
                    put_svarint(&mut p, 1);
                }
            } else {
                put_svarint(&mut p, ((v - prevpos) << 1) | 1);
                prevpos = v;
            }
        }
        if zeros > 0 {
            put_uvarint(&mut p, zeros << 1);
        }
        let plen = (p.len() - ptab_start) as u64;
        p.extend(plen.to_be_bytes());
        let mut tt = Vec::new();
        let mut prev = 0u64;
        for &t in &self.times {
            put_uvarint(&mut tt, t - prev);
            prev = t;
        }
        let tz = zlib(&tt, 9);
        let (tdata, tpacked) = if tz.len() < tt.len() { (&tz, tz.len()) } else { (&tt, tt.len()) };
        p.extend_from_slice(tdata);
        p.extend((tt.len() as u64).to_be_bytes());
        p.extend((tpacked as u64).to_be_bytes());
        p.extend((self.times.len() as u64).to_be_bytes());
        p[16..24].copy_from_slice(&memreq.to_be_bytes());
        self.out.write_all(&[block::VCDATA_DYN_ALIAS2]).map_err(io)?;
        self.out.write_all(&(p.len() as u64 + 8).to_be_bytes()).map_err(io)?;
        self.out.write_all(&p).map_err(io)?;
        for t in &mut self.tracks {
            t.entries.clear();
        }
        self.buffered = 0;
        self.sections += 1;
        Ok(())
    }

    fn write_block(&mut self, kind: u8, payload: &[u8]) -> Result<(), FstError> {
        self.out.write_all(&[kind]).map_err(io)?;
        self.out.write_all(&(payload.len() as u64 + 8).to_be_bytes()).map_err(io)?;
        self.out.write_all(payload).map_err(io)
    }

    /// Close the file: the last block, geometry, blackouts, hierarchy, and the header.
    pub fn finish(mut self) -> Result<W, FstError> {
        if !self.started {
            // Time never advanced: the values set are changes at time 0.
            self.time(0)?;
            self.firsttime = 0;
            self.block_start = 0;
            for i in 0..self.kinds.len() {
                let k = self.kinds[i];
                if k == Kind::VarLen {
                    continue;
                }
                let o = self.offsets[i];
                let v = self.curval[o..o + k.frame_len()].to_vec();
                self.tracks[i].entries.push((0, v));
            }
        }
        self.flush()?;
        // Geometry: per handle its width; 0 = real; all ones = variable length.
        let mut g = Vec::new();
        for k in &self.kinds {
            put_uvarint(
                &mut g,
                match k {
                    Kind::Bits(n) => *n as u64,
                    Kind::Real => 0,
                    Kind::VarLen => 0xFFFF_FFFF,
                },
            );
        }
        let gz = zlib(&g, 9);
        let gdata = if gz.len() < g.len() { gz } else { g.clone() };
        let mut gp = Vec::new();
        gp.extend((g.len() as u64).to_be_bytes());
        gp.extend((self.kinds.len() as u64).to_be_bytes());
        gp.extend(gdata);
        self.write_block(block::GEOM, &gp)?;
        if !self.blackouts.is_empty() {
            let mut bp = Vec::new();
            put_uvarint(&mut bp, self.blackouts.len() as u64);
            let mut prev = 0u64;
            for &(t, on) in &self.blackouts {
                bp.push(on as u8);
                put_uvarint(&mut bp, t - prev);
                prev = t;
            }
            self.write_block(block::BLACKOUT, &bp)?;
        }
        let mut hp = Vec::new();
        hp.extend((self.hier.len() as u64).to_be_bytes());
        hp.extend(gzip(&self.hier, 4));
        self.write_block(block::HIER, &hp)?;
        // The header, now that the counts are known.
        let mut h = Vec::with_capacity(HDR_PAYLOAD_LEN);
        h.extend(self.firsttime.to_be_bytes());
        h.extend(self.curtime.to_be_bytes());
        h.extend(ENDIAN_TEST.to_le_bytes());
        h.extend((self.opts.block_bytes as u64).to_be_bytes());
        h.extend(self.num_scopes.to_be_bytes());
        h.extend(self.num_vars.to_be_bytes());
        h.extend((self.kinds.len() as u64).to_be_bytes());
        h.extend(self.sections.to_be_bytes());
        h.push(self.opts.timescale as u8);
        let mut ver = self.opts.writer.as_bytes().to_vec();
        ver.truncate(HDR_VERSION_LEN - 1);
        ver.resize(HDR_VERSION_LEN, 0);
        h.extend(ver);
        let mut date = self.opts.date.clone().unwrap_or_else(now_date).into_bytes();
        date.truncate(HDR_DATE_LEN - 1);
        date.resize(HDR_DATE_LEN, 0);
        h.extend(date);
        h.push(self.opts.file_type);
        h.extend(self.opts.timezero.to_be_bytes());
        self.out.seek(SeekFrom::Start(9)).map_err(io)?;
        self.out.write_all(&h).map_err(io)?;
        self.out.seek(SeekFrom::End(0)).map_err(io)?;
        self.out.flush().map_err(io)?;
        Ok(self.out)
    }
}

/// Which signals and what time span a copy or export keeps.
#[derive(Debug, Clone, Default)]
pub struct Selection {
    /// Signal patterns (empty = all), matched against the full dotted name without its range.
    pub patterns: Vec<String>,
    pub exact: bool,
    pub regex: bool,
    pub start: u64,
    pub end: Option<u64>,
}

impl Selection {
    /// The name matcher this selection describes.
    pub fn matcher(&self) -> Result<crate::fst_query::Matcher, FstError> {
        crate::fst_query::Matcher::new(&self.patterns, self.regex, self.exact)
    }
}

/// The base name a query matches on: the stored name up to its first space (the range a
/// writer may append, `data [7:0]`, is dropped).
pub fn base_name(n: &str) -> &str {
    n.split(' ').next().unwrap_or(n)
}

/// Copy the selected signals of `r` within the selected window into `w`. Each kept signal's
/// value at the window start becomes its initial value. Returns the number of signals kept.
pub fn copy<W: Write + Seek>(
    r: &mut FstReader,
    w: &mut FstWriter<W>,
    sel: &Selection,
) -> Result<usize, FstError> {
    let m = sel.matcher()?;
    let end = sel.end.unwrap_or(u64::MAX);
    // Which handles are wanted: those of the variables whose full names match.
    let mut keep_h: HashMap<u32, u32> = HashMap::new(); // old handle -> new handle
    let mut keep_var = Vec::new();
    for (path, v) in r.vars() {
        let full = full_name_for_match(&path, &v.name);
        keep_var.push(m.matches(&full));
    }
    // The hierarchy, writing a scope only when a kept variable is inside it.
    let mut pending: Vec<(u8, String, String)> = Vec::new();
    let mut open = 0usize; // how many of the pending scopes are written
    let mut vi = 0usize;
    let mut kept = 0usize;
    let hier = r.hier.clone();
    for item in &hier {
        match item {
            Hier::Scope { kind, name, component } => pending.push((*kind, name.clone(), component.clone())),
            Hier::Upscope => {
                if open == pending.len() && open > 0 {
                    w.upscope();
                    open -= 1;
                }
                pending.pop();
            }
            Hier::AttrBegin { kind, sub, name, arg } => {
                if open == pending.len() {
                    w.attr_begin(*kind, *sub, name, *arg);
                }
            }
            Hier::AttrEnd => {
                if open == pending.len() {
                    w.attr_end();
                }
            }
            Hier::Var(v) => {
                let keep = keep_var.get(vi).copied().unwrap_or(false);
                vi += 1;
                if !keep {
                    continue;
                }
                while open < pending.len() {
                    let (k, n, c) = &pending[open];
                    w.scope(*k, n, c);
                    open += 1;
                }
                let alias = keep_h.get(&v.handle).copied();
                let nh = w.var(v.kind, v.dir, &v.name, v.len, alias);
                keep_h.entry(v.handle).or_insert(nh);
                kept += 1;
            }
        }
    }
    while open > 0 {
        w.upscope();
        open -= 1;
    }
    // Initial values: the first frame, then every change before the window start.
    let mut cur: HashMap<u32, Value> = HashMap::new();
    if let Some((_t0, frame)) = r.initial_frame()? {
        for &oh in keep_h.keys() {
            if let Some(v) = r.frame_value(&frame, oh) {
                cur.insert(oh, v);
            }
        }
    }
    let want = |h: u32| keep_h.contains_key(&h);
    let mut in_window: Vec<(u64, u32, Value)> = Vec::new();
    {
        let mut f = |c: &crate::fst_wave::Change| {
            if c.time < sel.start {
                cur.insert(c.handle, c.value.clone());
            } else {
                in_window.push((c.time, c.handle, c.value.clone()));
            }
        };
        r.for_each_change(&want, 0, end, &mut f)?;
    }
    let mut olds: Vec<u32> = keep_h.keys().copied().collect();
    olds.sort_unstable();
    for oh in &olds {
        if let Some(v) = cur.get(oh) {
            w.value(keep_h[oh], v)?;
        }
    }
    let mut last_t = None;
    for (t, oh, v) in in_window {
        if last_t != Some(t) {
            w.time(t)?;
            last_t = Some(t);
        }
        w.value(keep_h[&oh], &v)?;
    }
    if last_t.is_none() {
        w.time(sel.start.max(r.header.start_time))?;
    }
    Ok(kept)
}

/// The full name a pattern is matched against (dotted path, range dropped).
pub fn full_name_for_match(path: &str, stored: &str) -> String {
    let cut = path.len() - stored.len();
    format!("{}{}", &path[..cut], base_name(stored))
}

/// VCD → FST, streaming: declarations, then `#time` and value changes. Returns the number of
/// variables declared.
pub fn vcd_to_fst<W: Write + Seek>(
    input: &mut dyn BufRead,
    opts: WriterOptions,
    out: W,
) -> Result<(usize, W), FstError> {
    let mut toks = VcdTokens::new(input);
    let mut opts = opts;
    let mut ids: HashMap<String, u32> = HashMap::new();
    // Declarations are collected first: the timescale may follow them.
    let mut decls: Vec<Decl> = Vec::new();
    let mut nvars = 0usize;
    while let Some(t) = toks.next()? {
        match t.as_str() {
            "$timescale" => {
                let body = toks.until_end()?;
                opts.timescale = parse_timescale(&body.join(""));
            }
            "$date" => {
                let body = toks.until_end()?;
                if opts.date.is_none() && !body.is_empty() {
                    opts.date = Some(format!("{}\n", body.join(" ")));
                }
            }
            "$version" => {
                let _ = toks.until_end()?;
            }
            "$scope" => {
                let body = toks.until_end()?;
                let kind = body.first().and_then(|k| st::from_name(k)).unwrap_or(0);
                let name = body.get(1).cloned().unwrap_or_default();
                decls.push(Decl::Scope(kind, name));
            }
            "$upscope" => {
                let _ = toks.until_end()?;
                decls.push(Decl::Upscope);
            }
            "$var" => {
                let body = toks.until_end()?;
                if body.len() < 4 {
                    return err(format!("malformed $var: {}", body.join(" ")));
                }
                let kind = vt::from_name(&body[0]).unwrap_or(vt::WIRE);
                let len: u32 = body[1].parse().unwrap_or(1);
                let mut name = body[3].clone();
                if body.len() > 4 {
                    name.push(' ');
                    name.push_str(&body[4..].join(""));
                }
                decls.push(Decl::Var(kind, len, body[2].clone(), name));
                nvars += 1;
            }
            "$comment" => {
                let _ = toks.until_end()?;
            }
            "$enddefinitions" => {
                let _ = toks.until_end()?;
                break;
            }
            _ => {}
        }
    }
    let mut w = FstWriter::new(out, opts)?;
    for d in &decls {
        match d {
            Decl::Scope(k, n) => w.scope(*k, n, ""),
            Decl::Upscope => w.upscope(),
            Decl::Var(k, len, id, name) => {
                let alias = ids.get(id).copied();
                let h = w.var(*k, 0, name, *len, alias);
                ids.entry(id.clone()).or_insert(h);
            }
        }
    }
    while let Some(t) = toks.next()? {
        let b = t.as_bytes();
        match b[0] {
            b'#' => {
                let v: u64 = t[1..].parse().map_err(|_| FstError(format!("bad time {t}")))?;
                w.time(v)?;
            }
            b'$' => match t.as_str() {
                "$dumpoff" => w.dump_active(false),
                "$dumpon" => w.dump_active(true),
                "$comment" => {
                    let _ = toks.until_end()?;
                }
                _ => {}
            },
            b'b' | b'B' => {
                let val = t[1..].to_ascii_lowercase();
                let id = toks.next()?.ok_or_else(|| FstError("vector value without id".into()))?;
                if let Some(&h) = ids.get(&id) {
                    w.value(h, &Value::Bits(val.into_bytes()))?;
                }
            }
            b'r' | b'R' => {
                let d: f64 = t[1..].parse().unwrap_or(f64::NAN);
                let id = toks.next()?.ok_or_else(|| FstError("real value without id".into()))?;
                if let Some(&h) = ids.get(&id) {
                    w.value(h, &Value::Real(d))?;
                }
            }
            b's' | b'S' => {
                let s = t.as_bytes()[1..].to_vec();
                let id = toks.next()?.ok_or_else(|| FstError("string value without id".into()))?;
                if let Some(&h) = ids.get(&id) {
                    w.value(h, &Value::Str(s))?;
                }
            }
            c => {
                let c = c.to_ascii_lowercase();
                if b"01xzhuwl-?".contains(&c) {
                    if let Some(&h) = ids.get(&t[1..]) {
                        w.value(h, &Value::Bits(vec![c]))?;
                    }
                }
            }
        }
    }
    Ok((nvars, w.finish()?))
}

enum Decl {
    Scope(u8, String),
    Upscope,
    Var(u8, u32, String, String),
}

/// The power of ten a `$timescale` names (`1ns`, `10 ps`, `100us`).
pub fn parse_timescale(s: &str) -> i8 {
    let s = s.trim();
    let digits: String = s.chars().take_while(|c| c.is_ascii_digit()).collect();
    let unit = s[digits.len()..].trim();
    let base = match unit {
        "s" => 0,
        "ms" => -3,
        "us" => -6,
        "ns" => -9,
        "ps" => -12,
        "fs" => -15,
        _ => -9,
    };
    let extra = match digits.as_str() {
        "10" => 1,
        "100" => 2,
        _ => 0,
    };
    (base + extra) as i8
}

/// Whitespace-separated VCD tokens, read a line at a time.
struct VcdTokens<'a> {
    r: &'a mut dyn BufRead,
    line: Vec<String>,
    at: usize,
}

impl<'a> VcdTokens<'a> {
    fn new(r: &'a mut dyn BufRead) -> Self {
        VcdTokens { r, line: Vec::new(), at: 0 }
    }

    fn next(&mut self) -> Result<Option<String>, FstError> {
        loop {
            if self.at < self.line.len() {
                self.at += 1;
                return Ok(Some(std::mem::take(&mut self.line[self.at - 1])));
            }
            let mut s = String::new();
            if self.r.read_line(&mut s).map_err(io)? == 0 {
                return Ok(None);
            }
            self.line = s.split_whitespace().map(str::to_string).collect();
            self.at = 0;
        }
    }

    fn until_end(&mut self) -> Result<Vec<String>, FstError> {
        let mut v = Vec::new();
        while let Some(t) = self.next()? {
            if t == "$end" {
                return Ok(v);
            }
            v.push(t);
        }
        Ok(v)
    }
}

/// The largest export [`fst_to_vcd`] writes before it stops (bytes).
pub const VCD_EXPORT_LIMIT: u64 = 256 << 20;

/// A bounded FST → VCD export. ⛔ Requires a window (`sel.end`) and at least one pattern, and
/// stops at [`VCD_EXPORT_LIMIT`] bytes: converting a whole dump is what this refuses to do.
/// Returns (signals, changes written).
pub fn fst_to_vcd(r: &mut FstReader, sel: &Selection, out: &mut dyn Write) -> Result<(usize, u64), FstError> {
    let Some(end) = sel.end else {
        return err("a VCD export needs --end (and --start): exporting a whole dump is refused");
    };
    if sel.patterns.is_empty() {
        return err("a VCD export needs a signal list: exporting every signal is refused");
    }
    let m = sel.matcher()?;
    let vars = r.vars();
    let mut code_of: HashMap<u32, String> = HashMap::new();
    let mut n = 0usize;
    let mut text = String::new();
    text.push_str(&format!("$date\n\t{}\n$end\n", r.header.date));
    text.push_str(&format!("$version\n\t{} (exported by vyges-loom)\n$end\n", r.header.writer));
    text.push_str(&format!("$timescale\n\t{}\n$end\n", timescale_text(r.header.timescale)));
    let mut scope: Vec<String> = Vec::new();
    let mut written_depth = 0usize;
    let mut vi = 0usize;
    let hier = r.hier.clone();
    for item in &hier {
        match item {
            Hier::Scope { kind, name, .. } => scope.push(format!("{} {}", st::name(*kind), name)),
            Hier::Upscope => {
                if written_depth == scope.len() && written_depth > 0 {
                    text.push_str("$upscope $end\n");
                    written_depth -= 1;
                }
                scope.pop();
            }
            Hier::Var(v) => {
                let (path, _) = &vars[vi];
                vi += 1;
                if !m.matches(&full_name_for_match(path, &v.name)) {
                    continue;
                }
                while written_depth < scope.len() {
                    text.push_str(&format!("$scope {} $end\n", scope[written_depth]));
                    written_depth += 1;
                }
                let next = code_of.len() as u32;
                let code = code_of.entry(v.handle).or_insert_with(|| vcd_code(next)).clone();
                let (nm, range) = match v.name.split_once(' ') {
                    Some((a, b)) => (a.to_string(), format!(" {b}")),
                    None => (v.name.clone(), String::new()),
                };
                let len = match r.kind(v.handle) {
                    Some(Kind::Bits(w)) => w,
                    Some(Kind::Real) => 64,
                    _ => 1,
                };
                text.push_str(&format!("$var {} {} {} {}{} $end\n", vt::name(v.kind), len, code, nm, range));
                n += 1;
            }
            _ => {}
        }
    }
    while written_depth > 0 {
        text.push_str("$upscope $end\n");
        written_depth -= 1;
    }
    text.push_str("$enddefinitions $end\n");
    out.write_all(text.as_bytes()).map_err(io)?;
    let mut bytes = text.len() as u64;
    // Values at the window start, then the changes inside it.
    let mut cur: HashMap<u32, Value> = HashMap::new();
    if let Some((_, frame)) = r.initial_frame()? {
        for &h in code_of.keys() {
            if let Some(v) = r.frame_value(&frame, h) {
                cur.insert(h, v);
            }
        }
    }
    let want = |h: u32| code_of.contains_key(&h);
    let mut window: Vec<(u64, u32, Value)> = Vec::new();
    r.for_each_change(&want, 0, end, &mut |c| {
        if c.time < sel.start {
            cur.insert(c.handle, c.value.clone());
        } else {
            window.push((c.time, c.handle, c.value.clone()));
        }
    })?;
    let line = |h: u32, v: &Value, code: &str, kind: Option<Kind>| -> String {
        match (v, kind) {
            (Value::Bits(b), Some(Kind::Bits(1))) => format!("{}{}\n", String::from_utf8_lossy(b), code),
            (Value::Bits(b), _) => format!("b{} {}\n", String::from_utf8_lossy(b), code),
            (Value::Real(d), _) => format!("r{} {}\n", fmt_g16(*d), code),
            (Value::Str(s), _) => {
                let _ = h;
                format!("s{} {}\n", String::from_utf8_lossy(s), code)
            }
        }
    };
    let mut changes = 0u64;
    let mut s = format!("#{}\n$dumpvars\n", sel.start);
    let mut hs: Vec<u32> = cur.keys().copied().collect();
    hs.sort_unstable();
    for h in hs {
        s.push_str(&line(h, &cur[&h], &code_of[&h], r.kind(h)));
    }
    s.push_str("$end\n");
    out.write_all(s.as_bytes()).map_err(io)?;
    bytes += s.len() as u64;
    let mut last = Some(sel.start);
    for (t, h, v) in &window {
        let mut s = String::new();
        if last != Some(*t) {
            s.push_str(&format!("#{t}\n"));
            last = Some(*t);
        }
        s.push_str(&line(*h, v, &code_of[h], r.kind(*h)));
        bytes += s.len() as u64;
        if bytes > VCD_EXPORT_LIMIT {
            return err(format!(
                "VCD export stopped at {} MiB: narrow the window or the signal list",
                VCD_EXPORT_LIMIT >> 20
            ));
        }
        out.write_all(s.as_bytes()).map_err(io)?;
        changes += 1;
    }
    Ok((n, changes))
}

/// The VCD identifier code for the n-th signal (printable `!`..`~`, base 94).
pub fn vcd_code(mut n: u32) -> String {
    let mut s = String::new();
    loop {
        s.push((b'!' + (n % 94) as u8) as char);
        n /= 94;
        if n == 0 {
            return s;
        }
        n -= 1;
    }
}

/// `1ns`, `10ps`, … for a power of ten.
pub fn timescale_text(e: i8) -> String {
    let e = e as i32;
    let base = if e >= 0 { e / 3 * 3 } else { (e - 2) / 3 * 3 };
    let unit = match base {
        0 => "s",
        -3 => "ms",
        -6 => "us",
        -9 => "ns",
        -12 => "ps",
        -15 => "fs",
        _ => return format!("1e{e} s"),
    };
    format!("{}{}", 10i64.pow((e - base) as u32), unit)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn write_small(block_bytes: usize) -> Vec<u8> {
        let opts = WriterOptions { date: Some("Thu Jan  1 00:00:00 1970\n".into()), block_bytes, ..Default::default() };
        let mut w = FstWriter::new(Cursor::new(Vec::new()), opts).unwrap();
        w.scope(0, "top", "top");
        let clk = w.var(vt::WIRE, 1, "clk", 1, None);
        let bus = w.var(vt::REG, 2, "bus [11:0]", 12, None);
        let r = w.var(vt::REAL, 0, "v", 64, None);
        let s = w.var(vt::STRING, 0, "msg", 0, None);
        let al = w.var(vt::WIRE, 0, "clk_alias", 1, Some(clk));
        assert_eq!(al, clk);
        w.upscope();
        w.time(0).unwrap();
        w.value(clk, &Value::Bits(b"0".to_vec())).unwrap();
        w.value(bus, &Value::Bits(b"101".to_vec())).unwrap();
        w.value(r, &Value::Real(0.25)).unwrap();
        for t in 1..200u64 {
            w.time(t * 5).unwrap();
            w.value(clk, &Value::Bits(if t % 2 == 1 { b"1".to_vec() } else { b"0".to_vec() })).unwrap();
            if t % 7 == 0 {
                w.value(bus, &Value::Bits(format!("{:b}", t).into_bytes())).unwrap();
                w.value(s, &Value::Str(format!("m{t}").into_bytes())).unwrap();
            }
            if t == 50 {
                w.value(bus, &Value::Bits(b"x1z0".to_vec())).unwrap();
                w.value(r, &Value::Real(-1.5e-9)).unwrap();
            }
        }
        w.finish().unwrap().into_inner()
    }

    fn read_all(bytes: Vec<u8>) -> (FstReader, Vec<crate::fst_wave::Change>) {
        let mut r = FstReader::from_bytes(bytes).unwrap();
        let mut v = Vec::new();
        r.for_each_change(&|_| true, 0, u64::MAX, &mut |c| v.push(c.clone())).unwrap();
        (r, v)
    }

    /// What is written reads back the same, in one block or in many: the block boundary
    /// (a new frame, a time table starting at the last time) must not change any value.
    #[test]
    fn round_trip_one_and_many_blocks() {
        let (r1, one) = read_all(write_small(64 << 20));
        let (r2, many) = read_all(write_small(64));
        assert!(r2.vc_blocks().len() > 3, "small block size must split the dump");
        assert_eq!(r1.vc_blocks().len(), 1);
        // A block starts at the previous block's last time; nothing changes there twice.
        assert_eq!(one, many);
        assert_eq!(r1.header.end_time, 995);
        assert_eq!(r1.header.num_vars, 5);
        assert_eq!(r1.header.max_handle, 4);
        let clk: Vec<_> = one.iter().filter(|c| c.handle == 1).collect();
        assert_eq!(clk.len(), 200);
        let bus50 = one.iter().find(|c| c.handle == 2 && c.time == 250).unwrap();
        // A short value with a leading x extends left with x.
        assert_eq!(bus50.value, Value::Bits(fit_bits(b"x1z0", 12)));
        let r50 = one.iter().find(|c| c.handle == 3 && c.time == 250).unwrap();
        assert_eq!(r50.value, Value::Real(-1.5e-9));
        let s7 = one.iter().find(|c| c.handle == 4 && c.time == 35).unwrap();
        assert_eq!(s7.value, Value::Str(b"m7".to_vec()));
        // The bus's initial "101" extends left with zeros.
        let b0 = one.iter().find(|c| c.handle == 2 && c.time == 0).unwrap();
        assert_eq!(b0.value, Value::Bits(b"000000000101".to_vec()));
    }

    #[test]
    fn vcd_to_fst_reads_back() {
        let vcd = "$timescale 1ps $end\n$scope module t $end\n$var wire 1 ! a $end\n$var wire 4 \" d [3:0] $end\n$var real 64 # r $end\n$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n0!\nb0011 \"\nr1.5 #\n$end\n#10\n1!\nbx \"\n#20\n0!\n";
        let mut inp = std::io::BufReader::new(vcd.as_bytes());
        let (n, out) = vcd_to_fst(&mut inp, WriterOptions::default(), Cursor::new(Vec::new())).unwrap();
        assert_eq!(n, 3);
        let (r, ch) = read_all(out.into_inner());
        assert_eq!(r.header.timescale, -12);
        let names: Vec<String> = r.vars().into_iter().map(|(p, _)| p).collect();
        assert_eq!(names, ["t.a", "t.d [3:0]", "t.r"]);
        let d10 = ch.iter().find(|c| c.handle == 2 && c.time == 10).unwrap();
        assert_eq!(d10.value, Value::Bits(b"xxxx".to_vec()));
        assert_eq!(ch.iter().filter(|c| c.handle == 1).count(), 3);
    }

    #[test]
    fn asctime_known_dates() {
        assert_eq!(asctime(0), "Thu Jan  1 00:00:00 1970\n");
        assert_eq!(asctime(1_793_750_400), "Wed Nov  4 00:00:00 2026\n");
    }

    #[test]
    fn signed_varint_writer_matches_reader() {
        for v in [-1i64, 0, 1, 63, 64, -64, -65, 1 << 40, -(1 << 40)] {
            let mut o = Vec::new();
            put_svarint(&mut o, v);
            let mut q = 0;
            assert_eq!(crate::fst_wave::svarint(&o, &mut q).unwrap(), v);
        }
    }

    #[test]
    fn export_refuses_unbounded() {
        let mut r = FstReader::from_bytes(write_small(64 << 20)).unwrap();
        let mut out = Vec::new();
        let all = Selection { patterns: vec!["clk".into()], ..Default::default() };
        assert!(fst_to_vcd(&mut r, &all, &mut out).is_err(), "no window must be refused");
        let nosig = Selection { end: Some(100), ..Default::default() };
        assert!(fst_to_vcd(&mut r, &nosig, &mut out).is_err(), "no signals must be refused");
        let ok = Selection { patterns: vec!["clk".into()], start: 10, end: Some(30), ..Default::default() };
        let (n, c) = fst_to_vcd(&mut r, &ok, &mut out).unwrap();
        assert_eq!(n, 2); // clk and its alias share one code
        assert!(c > 0);
    }

    #[test]
    fn copy_keeps_window_and_subset() {
        let mut r = FstReader::from_bytes(write_small(64 << 20)).unwrap();
        let mut w = FstWriter::new(Cursor::new(Vec::new()), WriterOptions::default()).unwrap();
        let sel = Selection { patterns: vec!["top.bus".into()], exact: true, start: 100, end: Some(500), ..Default::default() };
        let kept = copy(&mut r, &mut w, &sel).unwrap();
        assert_eq!(kept, 1);
        let (r2, ch) = read_all(w.finish().unwrap().into_inner());
        assert_eq!(r2.vars().len(), 1);
        assert!(ch.iter().all(|c| c.time >= 100 && c.time <= 500 || c.time == 0));
    }
}
