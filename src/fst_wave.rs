//! Clean-room **FST waveform reader**: the whole file, not just activity — header, hierarchy
//! (scopes, variables, attributes, aliases), and every value change in time order, including
//! reals and variable-length strings. [`crate::fst`] is the activity counter built for power;
//! this module is the general reader the `fst` query command and the writer's round trips use.
//!
//! ⛔ **Do not convert an FST file to VCD to look at it.** A simulation's FST is compressed many
//! times over; the same dump as VCD text can fill a disk, and read into an agent's context it
//! exhausts that too. Query the FST directly (`vyges loom fst <file> --info`, `--list`, a time
//! window and a signal list) — this reader seeks to the blocks a query needs and never loads the
//! whole file.
//!
//! ## The file, as this reader takes it
//!
//! A file is a sequence of blocks `[type u8][length u64 BE][payload]`, where the length counts
//! itself (so the payload is `length - 8` bytes). The block kinds:
//!
//! | type | block | read as |
//! |---|---|---|
//! | 0 | header | times, counts, timescale, writer, date, file type, time zero |
//! | 1, 5, 8 | value-change data | one time span each; 8 has the compact alias encoding |
//! | 2 | blackout | `$dumpoff` / `$dumpon` times |
//! | 3 | geometry | each handle's width (0 = real, all ones = variable length) |
//! | 4 | hierarchy, gzip | the declarations |
//! | 6, 7 | hierarchy, LZ4 / LZ4 twice | the declarations |
//! | 254 | gzip wrapper | the whole file gzipped; inflated to a temporary file first |
//! | 255 | skip | a block still being written: the end of readable data |
//!
//! A value-change block holds, front to back: its start and end time and the reader memory it
//! needs (three u64), the **frame** (every handle's value at the block start, one ASCII byte per
//! bit, eight raw bytes for a real; zlib-packed when its two lengths differ), the handle count
//! and a pack byte (`Z` zlib, `4` LZ4, `F` FastLZ), the per-handle **chains**, the **position
//! table** that locates each chain (or names the handle whose chain it shares), then the
//! position table's length, the zlib time table, and three u64 (its two lengths and its count).
//!
//! A chain starts with a varint: zero means the rest is stored as is, otherwise it is the
//! unpacked length and the rest is packed. Each entry carries a time-INDEX delta from the
//! previous entry: a 1-bit signal packs it with the value (`0`/`1` in two bits, the other
//! seven levels `xzhuwl-` plus `?` in four), a wider one packs a flag saying whether the value
//! is bits (MSB first, padded at the end) or one ASCII byte per bit, and a variable-length one
//! is followed by the byte count and the bytes.
//!
//! Changes are reported ordered by time, then by handle — the order a query prints them in.

#![deny(clippy::unwrap_used)]
#![cfg_attr(test, allow(clippy::unwrap_used))]

use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};

use crate::fst::FstError;
use crate::lz4;

/// Block types.
pub mod block {
    pub const HDR: u8 = 0;
    pub const VCDATA: u8 = 1;
    pub const BLACKOUT: u8 = 2;
    pub const GEOM: u8 = 3;
    pub const HIER: u8 = 4;
    pub const VCDATA_DYN_ALIAS: u8 = 5;
    pub const HIER_LZ4: u8 = 6;
    pub const HIER_LZ4DUO: u8 = 7;
    pub const VCDATA_DYN_ALIAS2: u8 = 8;
    pub const ZWRAPPER: u8 = 254;
    pub const SKIP: u8 = 255;

    pub fn is_vc(t: u8) -> bool {
        t == VCDATA || t == VCDATA_DYN_ALIAS || t == VCDATA_DYN_ALIAS2
    }
}

/// Hierarchy record tags that are not variable types.
pub mod tag {
    pub const ATTR_BEGIN: u8 = 252;
    pub const ATTR_END: u8 = 253;
    pub const SCOPE: u8 = 254;
    pub const UPSCOPE: u8 = 255;
}

/// Variable types (the hierarchy tag of a variable record).
pub mod vt {
    pub const EVENT: u8 = 0;
    pub const INTEGER: u8 = 1;
    pub const PARAMETER: u8 = 2;
    pub const REAL: u8 = 3;
    pub const REAL_PARAMETER: u8 = 4;
    pub const REG: u8 = 5;
    pub const SUPPLY0: u8 = 6;
    pub const SUPPLY1: u8 = 7;
    pub const TIME: u8 = 8;
    pub const TRI: u8 = 9;
    pub const TRIAND: u8 = 10;
    pub const TRIOR: u8 = 11;
    pub const TRIREG: u8 = 12;
    pub const TRI0: u8 = 13;
    pub const TRI1: u8 = 14;
    pub const WAND: u8 = 15;
    pub const WIRE: u8 = 16;
    pub const WOR: u8 = 17;
    pub const PORT: u8 = 18;
    pub const SPARRAY: u8 = 19;
    pub const REALTIME: u8 = 20;
    pub const STRING: u8 = 21;
    pub const SV_BIT: u8 = 22;
    pub const SV_LOGIC: u8 = 23;
    pub const SV_INT: u8 = 24;
    pub const SV_SHORTINT: u8 = 25;
    pub const SV_LONGINT: u8 = 26;
    pub const SV_BYTE: u8 = 27;
    pub const SV_ENUM: u8 = 28;
    pub const SV_SHORTREAL: u8 = 29;
    pub const MAX: u8 = 29;

    /// The VCD keyword for each type, index = type.
    pub const NAMES: [&str; 30] = [
        "event", "integer", "parameter", "real", "real_parameter", "reg", "supply0", "supply1",
        "time", "tri", "triand", "trior", "trireg", "tri0", "tri1", "wand", "wire", "wor", "port",
        "sparray", "realtime", "string", "bit", "logic", "int", "shortint", "longint", "byte",
        "enum", "shortreal",
    ];

    /// The four types whose values are IEEE doubles.
    pub fn is_real(t: u8) -> bool {
        t == REAL || t == REAL_PARAMETER || t == REALTIME || t == SV_SHORTREAL
    }

    pub fn name(t: u8) -> &'static str {
        NAMES.get(t as usize).copied().unwrap_or("wire")
    }

    pub fn from_name(s: &str) -> Option<u8> {
        NAMES.iter().position(|n| *n == s).map(|i| i as u8)
    }
}

/// Scope types (the byte after a scope tag).
pub mod st {
    pub const NAMES: [&str; 22] = [
        "module", "task", "function", "begin", "fork", "generate", "struct", "union", "class",
        "interface", "package", "program", "vhdl_architecture", "vhdl_procedure", "vhdl_function",
        "vhdl_record", "vhdl_process", "vhdl_block", "vhdl_for_generate", "vhdl_if_generate",
        "vhdl_generate", "vhdl_package",
    ];

    pub fn name(t: u8) -> &'static str {
        NAMES.get(t as usize).copied().unwrap_or("module")
    }

    pub fn from_name(s: &str) -> Option<u8> {
        NAMES.iter().position(|n| *n == s).map(|i| i as u8)
    }
}

/// Variable directions.
pub mod vd {
    pub const NAMES: [&str; 6] = ["implicit", "input", "output", "inout", "buffer", "linkage"];

    pub fn name(d: u8) -> &'static str {
        NAMES.get(d as usize).copied().unwrap_or("implicit")
    }
}

/// The eight non-binary single-bit levels, by their 3-bit code.
pub const LEVELS: &[u8; 8] = b"xzhuwl-?";
/// The double a header stores natively so a reader can tell the writer's byte order.
pub const ENDIAN_TEST: f64 = std::f64::consts::E;
/// Header payload sizes (the block's length field adds 8).
pub const HDR_VERSION_LEN: usize = 128;
pub const HDR_DATE_LEN: usize = 119;
pub const HDR_PAYLOAD_LEN: usize = 8 * 8 + 1 + HDR_VERSION_LEN + HDR_DATE_LEN + 1 + 8;
/// Largest section a reader will inflate: a sanity cap against a corrupt length.
pub const MAX_INFLATE: u64 = 1 << 32;

/// The header block.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Header {
    pub start_time: u64,
    pub end_time: u64,
    /// Doubles in this file are little-endian (the endian test matched that way).
    pub little_endian_doubles: bool,
    pub writer_memory: u64,
    pub num_scopes: u64,
    pub num_vars: u64,
    pub max_handle: u64,
    pub num_sections: u64,
    /// Power of ten of the time unit (-9 = 1 ns).
    pub timescale: i8,
    pub writer: String,
    pub date: String,
    pub file_type: u8,
    pub timezero: i64,
}

/// One hierarchy record, in file order.
#[derive(Debug, Clone, PartialEq)]
pub enum Hier {
    Scope { kind: u8, name: String, component: String },
    Upscope,
    AttrBegin { kind: u8, sub: u8, name: String, arg: u64 },
    AttrEnd,
    Var(VarDecl),
}

/// A declared variable. Several may share one handle (aliases).
#[derive(Debug, Clone, PartialEq)]
pub struct VarDecl {
    pub kind: u8,
    pub dir: u8,
    /// As stored: a writer may append the range (`data [7:0]`).
    pub name: String,
    /// Bits; 8 for a real (its byte count); 0 for a variable-length string.
    pub len: u32,
    /// 1-based value handle.
    pub handle: u32,
    /// Declared as an alias of an earlier handle.
    pub is_alias: bool,
}

/// How a handle's values are stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Bits(u32),
    Real,
    VarLen,
}

impl Kind {
    /// Bytes the handle occupies in a frame.
    pub fn frame_len(self) -> usize {
        match self {
            Kind::Bits(n) => n as usize,
            Kind::Real => 8,
            Kind::VarLen => 0,
        }
    }
}

/// One value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// One ASCII level per bit, MSB first.
    Bits(Vec<u8>),
    Real(f64),
    /// A variable-length value's bytes.
    Str(Vec<u8>),
}

impl Value {
    /// The text a VCD-style listing shows: bits as levels, a real as `%.16g`, a string as is.
    pub fn text(&self) -> String {
        match self {
            Value::Bits(b) => String::from_utf8_lossy(b).into_owned(),
            Value::Real(d) => fmt_g16(*d),
            Value::Str(s) => String::from_utf8_lossy(s).into_owned(),
        }
    }
}

/// One value change.
#[derive(Debug, Clone, PartialEq)]
pub struct Change {
    pub time: u64,
    pub handle: u32,
    pub value: Value,
}

/// Where a block is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockRef {
    pub kind: u8,
    /// File offset of the payload (after the 9-byte block header).
    pub pos: u64,
    pub len: u64,
    /// For a value-change block, its time span.
    pub start: u64,
    pub end: u64,
}

/// One value-change block, decoded.
#[derive(Debug, Clone, Default)]
pub struct BlockData {
    pub start: u64,
    pub end: u64,
    pub times: Vec<u64>,
    /// Every handle's value at the block start, in frame layout (see [`FstReader::frame_value`]).
    pub frame: Vec<u8>,
    pub changes: Vec<Change>,
}

trait Src: Read + Seek {}
impl<T: Read + Seek> Src for T {}

/// An opened FST file: the block index, header and hierarchy are read; value changes are read
/// one block at a time, on request.
pub struct FstReader {
    src: Box<dyn Src>,
    /// The inflated copy of a gzip-wrapped file, removed when the reader is dropped.
    unwrapped: Option<std::path::PathBuf>,
    pub header: Header,
    pub blocks: Vec<BlockRef>,
    pub hier: Vec<Hier>,
    /// Per handle (index = handle - 1).
    pub kinds: Vec<Kind>,
    /// Frame offset per handle.
    pub frame_offsets: Vec<usize>,
    /// `$dumpon` (true) / `$dumpoff` (false) times.
    pub blackouts: Vec<(u64, bool)>,
    /// The file used a gzip wrapper.
    pub wrapped: bool,
}

impl Drop for FstReader {
    fn drop(&mut self) {
        if let Some(p) = self.unwrapped.take() {
            let _ = std::fs::remove_file(p);
        }
    }
}

fn err<T>(m: impl Into<String>) -> Result<T, FstError> {
    Err(FstError(m.into()))
}

fn io(e: std::io::Error) -> FstError {
    FstError(format!("io: {e}"))
}

/// Unsigned LEB128.
pub fn uvarint(b: &[u8], p: &mut usize) -> Result<u64, FstError> {
    let mut r: u64 = 0;
    let mut s = 0u32;
    loop {
        let Some(&x) = b.get(*p) else { return err("varint overrun") };
        *p += 1;
        if s < 64 {
            r |= ((x & 0x7f) as u64) << s;
        }
        if x & 0x80 == 0 {
            return Ok(r);
        }
        s += 7;
    }
}

/// Signed LEB128: sign taken from bit 6 of the last byte.
pub fn svarint(b: &[u8], p: &mut usize) -> Result<i64, FstError> {
    let mut r: u64 = 0;
    let mut s = 0u32;
    loop {
        let Some(&x) = b.get(*p) else { return err("varint overrun") };
        *p += 1;
        if s < 64 {
            r |= ((x & 0x7f) as u64) << s;
        }
        s += 7;
        if x & 0x80 == 0 {
            if s < 64 && x & 0x40 != 0 {
                r |= !0u64 << s;
            }
            return Ok(r as i64);
        }
    }
}

pub fn be_u64(b: &[u8], o: usize) -> Result<u64, FstError> {
    match b.get(o..o + 8) {
        Some(s) => Ok(u64::from_be_bytes(s.try_into().expect("an 8-byte slice"))),
        None => err("truncated u64"),
    }
}

fn cstr(b: &[u8], p: &mut usize) -> String {
    let start = (*p).min(b.len());
    let mut e = start;
    while e < b.len() && b[e] != 0 {
        e += 1;
    }
    let s = String::from_utf8_lossy(&b[start..e]).into_owned();
    *p = (e + 1).min(b.len().max(e + 1));
    s
}

/// A zero-terminated string from a fixed-size field.
fn fixed_str(b: &[u8]) -> String {
    let e = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..e]).into_owned()
}

/// Bytes a gzip member's header takes (RFC 1952), or None if this is not gzip.
pub fn gzip_header_len(b: &[u8]) -> Option<usize> {
    if b.len() < 10 || b[0] != 0x1f || b[1] != 0x8b || b[2] != 8 {
        return None;
    }
    let flg = b[3];
    let mut p = 10usize;
    if flg & 4 != 0 {
        let xlen = u16::from_le_bytes([*b.get(p)?, *b.get(p + 1)?]) as usize;
        p += 2 + xlen;
    }
    for bit in [8u8, 16] {
        if flg & bit != 0 {
            while *b.get(p)? != 0 {
                p += 1;
            }
            p += 1;
        }
    }
    if flg & 2 != 0 {
        p += 2;
    }
    (p <= b.len()).then_some(p)
}

/// Inflate zlib or gzip (told apart by the magic bytes).
pub fn inflate(b: &[u8]) -> Result<Vec<u8>, FstError> {
    if let Some(h) = gzip_header_len(b) {
        miniz_oxide::inflate::decompress_to_vec(&b[h..]).map_err(|e| FstError(format!("gzip: {e:?}")))
    } else {
        miniz_oxide::inflate::decompress_to_vec_zlib(b).map_err(|e| FstError(format!("zlib: {e:?}")))
    }
}

/// A section stored packed only when that made it smaller: equal lengths mean stored as is.
fn maybe_inflate(b: &[u8], unpacked: u64) -> Result<Vec<u8>, FstError> {
    if b.len() as u64 == unpacked {
        Ok(b.to_vec())
    } else {
        inflate(b)
    }
}

/// FastLZ (levels 1 and 2), the third chain packing a writer may choose.
pub fn fastlz_decompress(input: &[u8], expected: usize) -> Result<Vec<u8>, FstError> {
    let mut out: Vec<u8> = Vec::with_capacity(expected);
    if input.is_empty() {
        return Ok(out);
    }
    let level = (input[0] >> 5) + 1;
    let mut ip = 1usize;
    let mut ctrl = (input[0] & 31) as usize;
    let get = |ip: &mut usize| -> Result<usize, FstError> {
        let v = *input.get(*ip).ok_or_else(|| FstError("fastlz overrun".into()))? as usize;
        *ip += 1;
        Ok(v)
    };
    loop {
        if ctrl >= 32 {
            let mut len = (ctrl >> 5) - 1;
            let mut ofs = (ctrl & 31) << 8;
            if level == 1 {
                if len == 6 {
                    len += get(&mut ip)?;
                }
                ofs += get(&mut ip)?;
            } else {
                if len == 6 {
                    loop {
                        let c = get(&mut ip)?;
                        len += c;
                        if c != 255 {
                            break;
                        }
                    }
                }
                let c = get(&mut ip)?;
                ofs += c;
                if c == 255 && (ctrl & 31) == 31 {
                    ofs = (get(&mut ip)? << 8) + get(&mut ip)?;
                    ofs += 8191;
                }
            }
            let back = ofs + 1;
            if back > out.len() {
                return err("fastlz bad offset");
            }
            let start = out.len() - back;
            for k in 0..len + 3 {
                let c = out[start + k];
                out.push(c);
            }
        } else {
            let n = ctrl + 1;
            let lit = input.get(ip..ip + n).ok_or_else(|| FstError("fastlz overrun".into()))?;
            out.extend_from_slice(lit);
            ip += n;
        }
        if ip >= input.len() {
            break;
        }
        ctrl = get(&mut ip)?;
    }
    Ok(out)
}

/// `%.16g`, as C prints it.
pub fn fmt_g16(d: f64) -> String {
    fmt_g(d, 16)
}

/// C's `%.<prec>g`.
pub fn fmt_g(d: f64, prec: usize) -> String {
    if d.is_nan() {
        return if d.is_sign_negative() { "-nan".into() } else { "nan".into() };
    }
    if d.is_infinite() {
        return if d < 0.0 { "-inf".into() } else { "inf".into() };
    }
    let p = prec.max(1);
    if d == 0.0 {
        return if d.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    // The exponent after rounding to p significant digits.
    let e_str = format!("{:.*e}", p - 1, d);
    let (mant, exp) = e_str.split_once('e').expect("{:e} always prints an exponent");
    let x: i32 = exp.parse().expect("{:e} prints a decimal exponent");
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s.to_string()
        }
    };
    if x < -4 || x >= p as i32 {
        let m = trim(mant);
        let sign = if x < 0 { '-' } else { '+' };
        format!("{m}e{sign}{:02}", x.abs())
    } else {
        let decimals = (p as i32 - 1 - x).max(0) as usize;
        trim(&format!("{:.*}", decimals, d))
    }
}

/// Stream-inflate a gzip member from `r` into `w`.
fn gunzip_stream(r: &mut dyn Read, w: &mut dyn Write) -> Result<u64, FstError> {
    use miniz_oxide::inflate::stream::{inflate as step, InflateState};
    use miniz_oxide::{DataFormat, MZFlush, MZStatus};
    let mut head = vec![0u8; 0];
    let mut buf = vec![0u8; 1 << 16];
    // Read enough for the gzip header (it is short; names are rare in a wrapper).
    while gzip_header_len(&head).is_none() {
        let n = r.read(&mut buf).map_err(io)?;
        if n == 0 {
            return err("gzip wrapper: no header");
        }
        head.extend_from_slice(&buf[..n]);
        if head.len() > 1 << 20 {
            return err("gzip wrapper: header too long");
        }
    }
    let h = gzip_header_len(&head).expect("the loop above ended on a complete header");
    let mut pending: Vec<u8> = head[h..].to_vec();
    let mut state = InflateState::new_boxed(DataFormat::Raw);
    let mut out = vec![0u8; 1 << 18];
    let mut total = 0u64;
    let mut eof = false;
    loop {
        if pending.is_empty() && !eof {
            let n = r.read(&mut buf).map_err(io)?;
            if n == 0 {
                eof = true;
            } else {
                pending.extend_from_slice(&buf[..n]);
            }
        }
        let flush = if eof { MZFlush::Finish } else { MZFlush::None };
        let res = step(&mut state, &pending, &mut out, flush);
        pending.drain(..res.bytes_consumed);
        w.write_all(&out[..res.bytes_written]).map_err(io)?;
        total += res.bytes_written as u64;
        match res.status {
            Ok(MZStatus::StreamEnd) => return Ok(total),
            Ok(_) => {}
            Err(e) => {
                if eof && res.bytes_written == 0 && res.bytes_consumed == 0 {
                    return err(format!("gzip wrapper: {e:?}"));
                }
            }
        }
        if eof && res.bytes_written == 0 && res.bytes_consumed == 0 {
            return err("gzip wrapper: truncated");
        }
    }
}

impl FstReader {
    /// Open and index a file. Reads the header, geometry, blackouts and hierarchy; no values.
    pub fn open(path: &str) -> Result<FstReader, FstError> {
        let mut f = File::open(path).map_err(|e| FstError(format!("{path}: {e}")))?;
        let mut first = [0u8; 1];
        let n = f.read(&mut first).map_err(io)?;
        if n == 0 {
            return err(format!("{path}: empty file"));
        }
        if first[0] == block::ZWRAPPER {
            // [254][len u64][unpacked len u64][gzip]: inflate beside the system temp files.
            f.seek(SeekFrom::Start(17)).map_err(io)?;
            let tmp = std::env::temp_dir().join(format!(
                "vyges-fst-{}-{}.fst",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos())
                    .unwrap_or(0)
            ));
            {
                let mut w = std::io::BufWriter::new(File::create(&tmp).map_err(io)?);
                let mut r = std::io::BufReader::new(&mut f);
                if let Err(e) = gunzip_stream(&mut r, &mut w) {
                    let _ = std::fs::remove_file(&tmp);
                    return Err(e);
                }
                w.flush().map_err(io)?;
            }
            let g = File::open(&tmp).map_err(io)?;
            let mut rd = FstReader::from_src(Box::new(std::io::BufReader::new(g)))?;
            rd.unwrapped = Some(tmp);
            rd.wrapped = true;
            return Ok(rd);
        }
        f.seek(SeekFrom::Start(0)).map_err(io)?;
        FstReader::from_src(Box::new(std::io::BufReader::with_capacity(1 << 16, f)))
    }

    /// Open from bytes in memory.
    pub fn from_bytes(bytes: Vec<u8>) -> Result<FstReader, FstError> {
        if bytes.first() == Some(&block::ZWRAPPER) {
            let mut out = Vec::new();
            let mut r = std::io::Cursor::new(bytes.get(17..).unwrap_or(&[]).to_vec());
            gunzip_stream(&mut r, &mut out)?;
            let mut rd = FstReader::from_src(Box::new(std::io::Cursor::new(out)))?;
            rd.wrapped = true;
            return Ok(rd);
        }
        FstReader::from_src(Box::new(std::io::Cursor::new(bytes)))
    }

    fn read_at(&mut self, pos: u64, len: u64) -> Result<Vec<u8>, FstError> {
        if len > MAX_INFLATE {
            return err("section too large");
        }
        self.src.seek(SeekFrom::Start(pos)).map_err(io)?;
        let mut v = vec![0u8; len as usize];
        self.src.read_exact(&mut v).map_err(io)?;
        Ok(v)
    }

    fn from_src(mut src: Box<dyn Src>) -> Result<FstReader, FstError> {
        let file_len = src.seek(SeekFrom::End(0)).map_err(io)?;
        let mut blocks = Vec::new();
        let mut pos = 0u64;
        while pos + 9 <= file_len {
            src.seek(SeekFrom::Start(pos)).map_err(io)?;
            let mut h = [0u8; 9];
            src.read_exact(&mut h).map_err(io)?;
            let kind = h[0];
            let len = u64::from_be_bytes(h[1..9].try_into().expect("an 8-byte slice"));
            // A block still being written, a length never backpatched, or one that runs past
            // the end: what an interrupted dump looks like. What came before it is readable.
            if kind == block::SKIP || len < 8 || pos + 1 + len > file_len {
                break;
            }
            let mut b = BlockRef { kind, pos: pos + 9, len: len - 8, start: 0, end: 0 };
            if block::is_vc(kind) && b.len >= 16 {
                let mut t = [0u8; 16];
                src.read_exact(&mut t).map_err(io)?;
                b.start = u64::from_be_bytes(t[0..8].try_into().expect("an 8-byte slice"));
                b.end = u64::from_be_bytes(t[8..16].try_into().expect("an 8-byte slice"));
            }
            blocks.push(b);
            pos += 1 + len;
        }
        let mut rd = FstReader {
            src,
            unwrapped: None,
            header: Header::default(),
            blocks,
            hier: Vec::new(),
            kinds: Vec::new(),
            frame_offsets: Vec::new(),
            blackouts: Vec::new(),
            wrapped: false,
        };
        let Some(hb) = rd.blocks.iter().find(|b| b.kind == block::HDR).copied() else {
            return err("missing header block");
        };
        let hp = rd.read_at(hb.pos, hb.len.min(HDR_PAYLOAD_LEN as u64 + 64))?;
        rd.header = parse_header(&hp)?;
        let mut geom: Option<Vec<Kind>> = None;
        let blocks = rd.blocks.clone();
        for b in &blocks {
            match b.kind {
                block::GEOM => {
                    let p = rd.read_at(b.pos, b.len)?;
                    let unpacked = be_u64(&p, 0)?;
                    let maxh = be_u64(&p, 8)?;
                    let data = maybe_inflate(&p[16..], unpacked)?;
                    let mut q = 0usize;
                    let mut ks = Vec::with_capacity(maxh as usize);
                    for _ in 0..maxh {
                        let v = uvarint(&data, &mut q)?;
                        ks.push(match v {
                            0 => Kind::Real,
                            0xFFFF_FFFF => Kind::VarLen,
                            n => Kind::Bits(n as u32),
                        });
                    }
                    geom = Some(ks);
                }
                block::BLACKOUT => {
                    let p = rd.read_at(b.pos, b.len)?;
                    let mut q = 0usize;
                    let n = uvarint(&p, &mut q)?;
                    let mut t = 0u64;
                    for _ in 0..n {
                        let Some(&a) = p.get(q) else { break };
                        q += 1;
                        t = t.wrapping_add(uvarint(&p, &mut q)?);
                        rd.blackouts.push((t, a != 0));
                    }
                }
                block::HIER | block::HIER_LZ4 | block::HIER_LZ4DUO => {
                    let p = rd.read_at(b.pos, b.len)?;
                    let data = unpack_hier(b.kind, &p)?;
                    rd.hier = parse_hier(&data)?;
                }
                _ => {}
            }
        }
        // Handle storage: the geometry when the file has one, else the declarations.
        rd.kinds = match geom {
            Some(g) => g,
            None => rd
                .hier
                .iter()
                .filter_map(|h| match h {
                    Hier::Var(v) if !v.is_alias => Some(if vt::is_real(v.kind) {
                        Kind::Real
                    } else if v.len == 0 || v.kind == vt::STRING {
                        Kind::VarLen
                    } else {
                        Kind::Bits(v.len)
                    }),
                    _ => None,
                })
                .collect(),
        };
        let mut off = 0usize;
        rd.frame_offsets = rd
            .kinds
            .iter()
            .map(|k| {
                let o = off;
                off += k.frame_len();
                o
            })
            .collect();
        // A header never backpatched carries no span: take it from the blocks.
        if rd.header.start_time == 0 && rd.header.end_time == 0 {
            let vc: Vec<&BlockRef> = rd.blocks.iter().filter(|b| block::is_vc(b.kind)).collect();
            if let (Some(f), Some(l)) = (vc.first(), vc.last()) {
                rd.header.start_time = f.start;
                rd.header.end_time = l.end;
            }
        }
        Ok(rd)
    }

    /// The value-change blocks, in file order.
    pub fn vc_blocks(&self) -> Vec<BlockRef> {
        self.blocks.iter().filter(|b| block::is_vc(b.kind)).copied().collect()
    }

    /// The declared variables with their full dotted paths (scope names joined by `.`, the
    /// variable name as stored).
    pub fn vars(&self) -> Vec<(String, VarDecl)> {
        let mut out = Vec::new();
        let mut scope: Vec<&str> = Vec::new();
        for h in &self.hier {
            match h {
                Hier::Scope { name, .. } => scope.push(name),
                Hier::Upscope => {
                    scope.pop();
                }
                Hier::Var(v) => {
                    let mut p = scope.join(".");
                    if !p.is_empty() {
                        p.push('.');
                    }
                    p.push_str(&v.name);
                    out.push((p, v.clone()));
                }
                _ => {}
            }
        }
        out
    }

    /// The kind of a handle (1-based).
    pub fn kind(&self, handle: u32) -> Option<Kind> {
        self.kinds.get((handle as usize).wrapping_sub(1)).copied()
    }

    /// A handle's value in a frame.
    pub fn frame_value(&self, frame: &[u8], handle: u32) -> Option<Value> {
        let i = (handle as usize).checked_sub(1)?;
        let k = *self.kinds.get(i)?;
        let o = *self.frame_offsets.get(i)?;
        let b = frame.get(o..o + k.frame_len())?;
        Some(match k {
            Kind::Bits(_) => Value::Bits(b.to_vec()),
            Kind::Real => Value::Real(self.double(b)),
            Kind::VarLen => return None,
        })
    }

    fn double(&self, b: &[u8]) -> f64 {
        let a: [u8; 8] = b.try_into().unwrap_or([0; 8]);
        if self.header.little_endian_doubles {
            f64::from_le_bytes(a)
        } else {
            f64::from_be_bytes(a)
        }
    }

    /// Decode one value-change block: the changes of the handles `want` accepts, at times in
    /// `[tmin, tmax]`, ordered by time then handle. The frame is decoded when `with_frame`.
    pub fn read_block(
        &mut self,
        b: &BlockRef,
        want: &dyn Fn(u32) -> bool,
        tmin: u64,
        tmax: u64,
        with_frame: bool,
    ) -> Result<BlockData, FstError> {
        let p = self.read_at(b.pos, b.len)?;
        let n = p.len();
        if n < 24 + 24 {
            return err("short value-change block");
        }
        let start = be_u64(&p, 0)?;
        let end = be_u64(&p, 8)?;
        let mut q = 24usize;
        let frame_unpacked = uvarint(&p, &mut q)?;
        let frame_packed = uvarint(&p, &mut q)?;
        let _frame_maxh = uvarint(&p, &mut q)?;
        let fend = q.checked_add(frame_packed as usize).filter(|&e| e <= n);
        let Some(fend) = fend else { return err("frame overruns block") };
        let frame = if with_frame { maybe_inflate(&p[q..fend], frame_unpacked)? } else { Vec::new() };
        q = fend;
        let nwaves = uvarint(&p, &mut q)? as usize;
        let packtype = *p.get(q).ok_or_else(|| FstError("short block".into()))?;
        let vc_start = q; // chain offsets count from the pack byte
        // The tail: time table, then the position table before it.
        let t_unpacked = be_u64(&p, n - 24)?;
        let t_packed = be_u64(&p, n - 16)? as usize;
        let t_count = be_u64(&p, n - 8)? as usize;
        let Some(t_start) = (n - 24).checked_sub(t_packed) else { return err("time table overruns") };
        let tdata = maybe_inflate(&p[t_start..n - 24], t_unpacked)?;
        let mut times = Vec::with_capacity(t_count.min(tdata.len()));
        let mut tq = 0usize;
        let mut acc = 0u64;
        for _ in 0..t_count {
            if tq >= tdata.len() {
                break;
            }
            acc = acc.wrapping_add(uvarint(&tdata, &mut tq)?);
            times.push(acc);
        }
        let Some(plen_at) = t_start.checked_sub(8) else { return err("position table overruns") };
        let plen = be_u64(&p, plen_at)? as usize;
        let Some(pstart) = plen_at.checked_sub(plen) else { return err("position table overruns") };
        if pstart < vc_start {
            return err("position table overlaps the chains");
        }
        let chains = decode_positions(b.kind, &p[pstart..plen_at], nwaves, pstart - vc_start)?;
        let mut changes = Vec::new();
        for (i, c) in chains.iter().enumerate() {
            let h = i as u32 + 1;
            if !want(h) {
                continue;
            }
            let Some((off, len)) = c else { continue };
            let Some(kind) = self.kind(h) else { continue };
            let s = vc_start + off;
            let Some(raw) = p.get(s..s + len) else { return err("chain overruns block") };
            let data = unpack_chain(raw, packtype)?;
            self.decode_chain(&data, kind, h, &times, tmin, tmax, &mut changes)?;
        }
        changes.sort_by(|a, b| a.time.cmp(&b.time).then(a.handle.cmp(&b.handle)));
        Ok(BlockData { start, end, times, frame, changes })
    }

    #[allow(clippy::too_many_arguments)]
    fn decode_chain(
        &self,
        d: &[u8],
        kind: Kind,
        h: u32,
        times: &[u64],
        tmin: u64,
        tmax: u64,
        out: &mut Vec<Change>,
    ) -> Result<(), FstError> {
        let mut q = 0usize;
        let mut ti = 0usize;
        while q < d.len() {
            let v = uvarint(d, &mut q)?;
            let value = match kind {
                Kind::Bits(1) => {
                    if v & 1 == 0 {
                        ti += (v >> 2) as usize;
                        Value::Bits(vec![b'0' + ((v >> 1) & 1) as u8])
                    } else {
                        ti += (v >> 4) as usize;
                        Value::Bits(vec![LEVELS[((v >> 1) & 7) as usize]])
                    }
                }
                Kind::Bits(w) => {
                    ti += (v >> 1) as usize;
                    let w = w as usize;
                    if v & 1 == 0 {
                        let nb = w.div_ceil(8);
                        let Some(bytes) = d.get(q..q + nb) else { return err("chain value overrun") };
                        q += nb;
                        Value::Bits((0..w).map(|j| b'0' + ((bytes[j / 8] >> (7 - (j % 8))) & 1)).collect())
                    } else {
                        let Some(bytes) = d.get(q..q + w) else { return err("chain value overrun") };
                        q += w;
                        Value::Bits(bytes.to_vec())
                    }
                }
                Kind::Real => {
                    ti += (v >> 1) as usize;
                    if v & 1 == 0 {
                        // A double whose eight bytes were all ASCII 0/1 went out as bits.
                        let Some(&byte) = d.get(q) else { return err("chain value overrun") };
                        q += 1;
                        let chars: Vec<u8> = (0..8).map(|j| b'0' + ((byte >> (7 - j)) & 1)).collect();
                        Value::Real(self.double(&chars))
                    } else {
                        let Some(bytes) = d.get(q..q + 8) else { return err("chain value overrun") };
                        q += 8;
                        Value::Real(self.double(bytes))
                    }
                }
                Kind::VarLen => {
                    ti += (v >> 1) as usize;
                    let l = uvarint(d, &mut q)? as usize;
                    let Some(bytes) = d.get(q..q + l) else { return err("chain value overrun") };
                    q += l;
                    Value::Str(bytes.to_vec())
                }
            };
            let Some(&t) = times.get(ti) else { break };
            if t > tmax {
                break;
            }
            if t >= tmin {
                out.push(Change { time: t, handle: h, value });
            }
        }
        Ok(())
    }

    /// Every change of the handles `want` accepts in `[tmin, tmax]`, block by block, to `f`.
    pub fn for_each_change(
        &mut self,
        want: &dyn Fn(u32) -> bool,
        tmin: u64,
        tmax: u64,
        f: &mut dyn FnMut(&Change),
    ) -> Result<(), FstError> {
        for b in self.vc_blocks() {
            if b.end < tmin {
                continue;
            }
            if b.start > tmax {
                break;
            }
            let d = self.read_block(&b, want, tmin, tmax, false)?;
            for c in &d.changes {
                f(c);
            }
        }
        Ok(())
    }

    /// The first value-change block's frame and start time: every handle's initial value.
    pub fn initial_frame(&mut self) -> Result<Option<(u64, Vec<u8>)>, FstError> {
        let Some(b) = self.vc_blocks().first().copied() else { return Ok(None) };
        let d = self.read_block(&b, &|_| false, 0, 0, true)?;
        Ok(Some((d.start, d.frame)))
    }
}

fn parse_header(p: &[u8]) -> Result<Header, FstError> {
    if p.len() < 8 * 8 + 1 + HDR_VERSION_LEN + HDR_DATE_LEN {
        return err("short header");
    }
    let et: [u8; 8] = p[16..24].try_into().expect("an 8-byte slice");
    let little = f64::from_le_bytes(et) == ENDIAN_TEST;
    let v0 = 65;
    let d0 = v0 + HDR_VERSION_LEN;
    let f0 = d0 + HDR_DATE_LEN;
    let mut date = fixed_str(&p[d0..f0]);
    while date.ends_with(['\n', '\r', ' ']) {
        date.pop();
    }
    Ok(Header {
        start_time: be_u64(p, 0)?,
        end_time: be_u64(p, 8)?,
        little_endian_doubles: little,
        writer_memory: be_u64(p, 24)?,
        num_scopes: be_u64(p, 32)?,
        num_vars: be_u64(p, 40)?,
        max_handle: be_u64(p, 48)?,
        num_sections: be_u64(p, 56)?,
        timescale: p[64] as i8,
        writer: fixed_str(&p[v0..d0]),
        date,
        file_type: p.get(f0).copied().unwrap_or(0),
        timezero: p.get(f0 + 1..f0 + 9).map(|s| i64::from_be_bytes(s.try_into().expect("an 8-byte slice"))).unwrap_or(0),
    })
}

/// A hierarchy payload: unpacked length (u64), then gzip, LZ4, or LZ4 twice (with the
/// once-packed length as a varint first).
fn unpack_hier(kind: u8, p: &[u8]) -> Result<Vec<u8>, FstError> {
    let unpacked = be_u64(p, 0)? as usize;
    let body = &p[8..];
    match kind {
        block::HIER => inflate(body),
        block::HIER_LZ4 => lz4::decompress(body, unpacked).map_err(FstError),
        _ => {
            let mut q = 0usize;
            let once = uvarint(body, &mut q)? as usize;
            let mid = lz4::decompress(&body[q..], once).map_err(FstError)?;
            lz4::decompress(&mid, unpacked).map_err(FstError)
        }
    }
}

/// The hierarchy records.
pub fn parse_hier(d: &[u8]) -> Result<Vec<Hier>, FstError> {
    let mut out = Vec::new();
    let mut q = 0usize;
    let mut next = 1u32;
    while q < d.len() {
        let t = d[q];
        q += 1;
        match t {
            tag::SCOPE => {
                let kind = *d.get(q).unwrap_or(&0);
                q += 1;
                let name = cstr(d, &mut q);
                let component = cstr(d, &mut q);
                out.push(Hier::Scope { kind, name, component });
            }
            tag::UPSCOPE => out.push(Hier::Upscope),
            tag::ATTR_BEGIN => {
                let kind = *d.get(q).unwrap_or(&0);
                let sub = *d.get(q + 1).unwrap_or(&0);
                q += 2;
                let name = cstr(d, &mut q);
                let arg = uvarint(d, &mut q)?;
                out.push(Hier::AttrBegin { kind, sub, name, arg });
            }
            tag::ATTR_END => out.push(Hier::AttrEnd),
            k if k <= vt::MAX => {
                let dir = *d.get(q).unwrap_or(&0);
                q += 1;
                let name = cstr(d, &mut q);
                let len = uvarint(d, &mut q)? as u32;
                let alias = uvarint(d, &mut q)? as u32;
                let (handle, is_alias) = if alias == 0 {
                    next += 1;
                    (next - 1, false)
                } else {
                    (alias, true)
                };
                out.push(Hier::Var(VarDecl { kind: k, dir, name, len, handle, is_alias }));
            }
            other => return err(format!("unknown hierarchy record {other}")),
        }
    }
    Ok(out)
}

/// The position table → per handle (index = handle - 1) the chain's (offset from the pack byte,
/// length), or None when the handle has no changes in the block.
fn decode_positions(
    kind: u8,
    t: &[u8],
    nwaves: usize,
    chains_end: usize,
) -> Result<Vec<Option<(usize, usize)>>, FstError> {
    // Per handle: Data(offset) | Alias(target index) | None.
    #[derive(Clone, Copy)]
    enum E {
        None,
        Data(usize),
        Alias(usize),
    }
    let mut e = vec![E::None; nwaves];
    let mut idx = 0usize;
    let mut pval: i64 = 0;
    let mut q = 0usize;
    let mut prev_alias: i64 = 0;
    let alias_to = |v: i64, idx: usize| -> E {
        // v = -(1-based handle of the chain's owner)
        let target = (-v - 1) as usize;
        if v < 0 && target < idx {
            E::Alias(target)
        } else {
            E::None
        }
    };
    while q < t.len() && idx < nwaves {
        if kind == block::VCDATA_DYN_ALIAS2 {
            if t[q] & 1 != 0 {
                let sh = svarint(t, &mut q)? >> 1;
                if sh > 0 {
                    pval += sh;
                    e[idx] = E::Data(pval as usize);
                } else if sh < 0 {
                    prev_alias = sh;
                    e[idx] = alias_to(sh, idx);
                } else {
                    e[idx] = alias_to(prev_alias, idx);
                }
                idx += 1;
            } else {
                let v = uvarint(t, &mut q)?;
                let run = (v >> 1) as usize;
                idx = (idx + run).min(nwaves);
            }
        } else {
            let v = uvarint(t, &mut q)?;
            if v == 0 {
                let a = uvarint(t, &mut q)? as i64;
                e[idx] = alias_to(-a, idx);
                idx += 1;
            } else if v & 1 != 0 {
                pval += (v >> 1) as i64;
                e[idx] = E::Data(pval as usize);
                idx += 1;
            } else {
                idx = (idx + (v >> 1) as usize).min(nwaves);
            }
        }
    }
    // Lengths: to the next chain, the last one to the position table.
    let mut offs: Vec<usize> = e.iter().filter_map(|x| if let E::Data(o) = x { Some(*o) } else { None }).collect();
    offs.sort_unstable();
    let len_of = |o: usize| -> usize {
        match offs.binary_search(&o) {
            Ok(k) if k + 1 < offs.len() => offs[k + 1] - o,
            _ => chains_end.saturating_sub(o),
        }
    };
    let mut out: Vec<Option<(usize, usize)>> = vec![None; nwaves];
    for i in 0..nwaves {
        out[i] = match e[i] {
            E::None => None,
            E::Data(o) => Some((o, len_of(o))),
            E::Alias(t) => out[t],
        };
    }
    Ok(out)
}

/// A chain: varint 0 + stored bytes, or varint unpacked length + packed bytes.
fn unpack_chain(raw: &[u8], packtype: u8) -> Result<Vec<u8>, FstError> {
    let mut q = 0usize;
    let un = uvarint(raw, &mut q)? as usize;
    let body = &raw[q..];
    if un == 0 {
        return Ok(body.to_vec());
    }
    match packtype {
        b'4' => lz4::decompress(body, un).map_err(FstError),
        b'F' => fastlz_decompress(body, un),
        _ => inflate(body),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn varints_round_trip_both_signs() {
        // Signed LEB128 takes its sign from bit 6 of the LAST byte, not from the value's width:
        // -1 is one byte (0x7f) and 63 is one byte (0x3f), 64 needs two.
        for (v, bytes) in [(-1i64, vec![0x7f]), (63, vec![0x3f]), (64, vec![0xc0, 0x00]), (-65, vec![0xbf, 0x7f])] {
            let mut q = 0;
            assert_eq!(svarint(&bytes, &mut q).unwrap(), v);
            assert_eq!(q, bytes.len());
        }
    }

    #[test]
    fn g16_matches_c() {
        assert_eq!(fmt_g16(0.5), "0.5");
        assert_eq!(fmt_g16(1e20), "1e+20");
        assert_eq!(fmt_g16(3.0), "3");
        assert_eq!(fmt_g16(-2.5e-7), "-2.5e-07");
        assert_eq!(fmt_g16(std::f64::consts::PI), "3.141592653589793");
        assert_eq!(fmt_g(1234567.0, 6), "1.23457e+06");
        assert_eq!(fmt_g(0.0001, 6), "0.0001");
    }

    #[test]
    fn fastlz_literal_and_match() {
        // Level 1: a 3-byte literal run "abc", then a match of length 3+3 at distance 3.
        // ctrl = (len-1+1)<<5 | ofs_hi: len field = 3 → 3+3-... encoded (3+1)<<5? Build directly:
        // literal ctrl 2 (3 bytes), match ctrl ((6-3)+1)<<5 = 128 with ofs low 2 (distance 3).
        let input = [2u8, b'a', b'b', b'c', 128, 2];
        let out = fastlz_decompress(&input, 9).unwrap();
        assert_eq!(out, b"abcabcabc");
    }

    #[test]
    fn existing_fixture_reads() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/counter.fst");
        let mut r = FstReader::open(path).unwrap();
        assert!(!r.vars().is_empty());
        let mut n = 0;
        r.for_each_change(&|_| true, 0, u64::MAX, &mut |_| n += 1).unwrap();
        assert!(n > 0);
    }
}
