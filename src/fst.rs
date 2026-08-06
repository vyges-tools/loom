//! Clean-room **FST** (GTKWave binary waveform) activity reader — the binary,
//! large-dump counterpart to [`crate::vcd`], producing the same [`NetIndex`] so a
//! consumer's `toggle_rate` is source-agnostic.
//!
//! Behind the `fst` feature (needs a zlib decoder for the time table; LZ4 is
//! clean-roomed in [`crate::lz4`]). Scope of this reader: **activity extraction** —
//! per-net toggle counts — not waveform replay.
//!
//! ## Format handled (validated against `verilator --trace-fst`, Verilator 5.040)
//! Blocks are `[type:u8][len:u64 BE][payload]`. We read `HDR(0)` (timescale + end
//! time), `HIER_LZ4(6)` (LZ4-compressed hierarchy → scopes/vars/handles), and
//! `VCDATA_DYN_ALIAS2(8)` (the dynamic-alias value-change block). In a VC block:
//! a front section (times, frame, waves-count, packtype), a `waves_data` region of
//! per-handle chains, and — read from the block *end* backward — a position/chain
//! table and a zlib time table. Each handle's chain is `varint(unclen) + LZ4` (or raw
//! when `unclen==0`); 1-bit chains are one varint per change, multi-bit chains are
//! `varint(tdelta<<1 | has_xz)` + packed BE bytes (or ASCII when `has_xz`). Toggle
//! counting mirrors the VCD reader: scalar = value transitions; vector = per-bit
//! Hamming distance between consecutive values.
//!
//! ## Multi-bit signals: fixed 2026-08-06, and how
//!
//! Values are packed **MSB-aligned**: a 3-bit value `011` is the byte `0110_0000`, padded at the
//! END. Reading from the low end returned the padding, so every signal whose width is not a
//! multiple of 8 decoded as all-zero and reported NO activity. Widths that ARE a multiple of 8
//! are identical either way, which is why every 8-, 16- and 32-bit bus looked correct and the
//! defect survived a cross-check against the same waveforms in VCD form.
//!
//! Verify with a waveform in both formats, GTKWave's `vcd2fst` supplying the second:
//!
//! ```sh
//! python3 tools/gen_vcd.py /tmp/vcdgen 200
//! for f in /tmp/vcdgen/*.vcd; do vcd2fst "$f" "${f%.vcd}.fst"; done
//! VYGES_CORPUS=/tmp/vcdgen cargo test --features fst --test corpus counts_known
//! ```
//!
//! Both readers now agree exactly: on all 200 generated dumps, on 42 real waveforms (Verilog,
//! SystemVerilog and VHDL), and on 8 Verilator-written triples. Four defects had to be fixed to
//! get there, each documented at the code that fixes it: the position table's varints were
//! sign-extended by the VALUE's magnitude instead of the ENCODING's byte count, a zero chain
//! entry ("repeat the previous alias") was read as a data offset, real-typed variables were
//! bit-expanded and named after a mantissa byte, and the nine-state levels `H`/`L` were counted
//! inconsistently between the two readers.

use std::collections::HashMap;

use crate::lz4;
use crate::names::NetIndex;
use crate::vcd::{build_sig, level, Sig};

#[derive(Debug, Clone, Default)]
pub struct Fst {
    pub idx: NetIndex,
    pub sim_time_s: f64,
}

#[derive(Debug)]
pub struct FstError(pub String);
impl std::fmt::Display for FstError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "fst error: {}", self.0)
    }
}
impl std::error::Error for FstError {}

impl Fst {
    /// Transitions / second for a netlist net (scope-aware; 0 if unresolved).
    pub fn toggle_rate(&self, net: &str) -> f64 {
        match self.idx.resolve(net) {
            Some(n) if self.sim_time_s > 0.0 => n as f64 / self.sim_time_s,
            _ => 0.0,
        }
    }

    pub fn with_scope(mut self, scope: Option<String>) -> Self {
        self.idx.scope = scope;
        self
    }

    pub fn collisions(&self) -> usize {
        self.idx.collisions()
    }

    pub fn load(path: &str) -> Result<Fst, FstError> {
        Fst::load_scoped(path, None, None)
    }

    /// Load with an optional `[from, to)` activity window (seconds) and design scope.
    pub fn load_scoped(
        path: &str,
        window: Option<(f64, Option<f64>)>,
        scope: Option<String>,
    ) -> Result<Fst, FstError> {
        let bytes = std::fs::read(path).map_err(|e| FstError(format!("{path}: {e}")))?;
        Ok(Fst::parse(&bytes, window)?.with_scope(scope))
    }

    pub fn parse(bytes: &[u8], window: Option<(f64, Option<f64>)>) -> Result<Fst, FstError> {
        // ---- split top-level blocks -------------------------------------------------
        let mut hdr: Option<&[u8]> = None;
        let mut hier: Option<(u8, &[u8])> = None;
        let mut vcdata: Vec<&[u8]> = Vec::new();
        let mut i = 0usize;
        while i + 9 <= bytes.len() {
            let t = bytes[i];
            let len = be_u64(bytes, i + 1)? as usize;
            let payload_start = i + 9;
            let payload_end = i + 1 + len;
            if payload_end > bytes.len() || payload_end < payload_start {
                return Err(FstError("block length overruns file".into()));
            }
            let payload = &bytes[payload_start..payload_end];
            match t {
                0 => hdr = Some(payload),
                4 | 6 | 7 => hier = Some((t, payload)),
                8 | 1 | 5 => vcdata.push(payload),
                _ => {}
            }
            i = payload_end;
        }
        let hdr = hdr.ok_or_else(|| FstError("missing header block".into()))?;
        let (htype, hpayload) = hier.ok_or_else(|| FstError("missing hierarchy block".into()))?;

        // ---- header: timescale exponent + end time ---------------------------------
        // payload offsets (block offset - 9): end_time @8, timescale i8 @64.
        let end_time = be_u64(hdr, 8)?;
        let ts_exp = *hdr.get(64).ok_or_else(|| FstError("short header".into()))? as i8;
        let timescale_s = 10f64.powi(ts_exp as i32);
        // Rate denominator: the full dump, or the window duration when windowed (so a
        // windowed FST rate is defined the same way as a windowed VCD rate).
        let end_s = end_time as f64 * timescale_s;
        let sim_time_s = match window {
            None => end_s,
            Some((f, t)) => {
                let eff_from = f.clamp(0.0, end_s);
                let eff_to = t.unwrap_or(end_s).clamp(eff_from, end_s);
                eff_to - eff_from
            }
        };

        // ---- hierarchy: scopes / vars / handles ------------------------------------
        let mut idx = NetIndex::default();
        let handles = parse_hierarchy(htype, hpayload, &mut idx)?;

        // ---- value changes ---------------------------------------------------------
        // chain_counts[handle] = toggle counts. Scalars: Vec of len 1 (the net's count).
        // Vectors: Vec of per-bit counts (MSB..LSB, matching build_sig's bit order).
        let n_handles = handles.iter().map(|h| h.handle + 1).max().unwrap_or(0);
        let mut chain_counts: Vec<Vec<u64>> = vec![Vec::new(); n_handles];
        for payload in &vcdata {
            decode_vc_block(payload, &handles, window, timescale_s, &mut chain_counts)?;
        }

        // ---- attribute counts to nets ----------------------------------------------
        for h in &handles {
            if h.ty == "string" {
                continue;
            }
            let sig = build_sig(&h.ty, h.width, &h.full_path, h.range.as_deref(), &mut idx);
            let counts = &chain_counts[h.handle];
            match sig {
                Sig::Scalar(full) => {
                    if let Some(&c) = counts.first() {
                        if c > 0 {
                            idx.add_toggles(&full, c);
                        }
                    }
                }
                Sig::Vector { bits } => {
                    for (bit, &c) in bits.iter().zip(counts.iter()) {
                        if c > 0 {
                            idx.add_toggles(bit, c);
                        }
                    }
                }
            }
        }

        Ok(Fst { idx, sim_time_s })
    }
}

/// One declared signal name in the hierarchy → its netlist path, width, and the
/// physical chain handle it reads from.
struct HierVar {
    full_path: String,
    ty: String,
    width: usize,
    range: Option<String>,
    handle: usize,
}

/// Decompress + walk the hierarchy block into declared vars. `idx` is not populated
/// here (that happens after chain decode, via `build_sig`), but paths are returned.
fn parse_hierarchy(
    htype: u8,
    payload: &[u8],
    _idx: &mut NetIndex,
) -> Result<Vec<HierVar>, FstError> {
    let unclen = be_u64(payload, 0)? as usize;
    let mut o = 8usize;
    let comp = if htype == 7 {
        // LZ4DUO: a varint "compressed once" length precedes the twice-compressed data.
        let (_once, no) = uvarint(payload, o)?;
        o = no;
        &payload[o..]
    } else {
        &payload[o..]
    };
    let data = match htype {
        6 | 7 => lz4::decompress(comp, unclen).map_err(FstError)?,
        4 => zlib(comp).map_err(FstError)?,
        _ => {
            return Err(FstError(format!(
                "unsupported hierarchy block type {htype}"
            )))
        }
    };

    let mut scope: Vec<String> = Vec::new();
    let mut vars: Vec<HierVar> = Vec::new();
    let mut next_handle = 0usize;
    let mut p = 0usize;
    while p < data.len() {
        let tag = data[p];
        p += 1;
        match tag {
            254 => {
                // scope begin: type u8, name\0, component\0
                let _stype = data.get(p).copied().unwrap_or(0);
                p += 1;
                let name = cstr(&data, &mut p);
                let _component = cstr(&data, &mut p);
                scope.push(name);
            }
            255 => {
                scope.pop();
            }
            252 => {
                // attribute begin: type u8, subtype u8, name\0, varint value
                p += 2;
                let _name = cstr(&data, &mut p);
                let (_v, np) = uvarint(&data, p)?;
                p = np;
            }
            253 => {} // attribute end
            0..=29 => {
                // variable: direction u8, name\0, varint length, varint alias
                let _dir = data.get(p).copied().unwrap_or(0);
                p += 1;
                let name = cstr(&data, &mut p);
                let (length, np) = uvarint(&data, p)?;
                p = np;
                let (alias, np2) = uvarint(&data, p)?;
                p = np2;
                let handle = if alias == 0 {
                    let h = next_handle;
                    next_handle += 1;
                    h
                } else {
                    (alias - 1) as usize
                };
                // Verilator embeds the range in the name: "i [31:0]". Split it off so the
                // base ("i") + range feed build_sig, matching the VCD reader's paths.
                let (vname, range) = match name.find(" [") {
                    Some(pos) => (name[..pos].to_string(), Some(name[pos + 1..].to_string())),
                    None => (name, None),
                };
                let full_path = if scope.is_empty() {
                    vname
                } else {
                    format!("{}.{}", scope.join("."), vname)
                };
                let width = if length == 0 || length == 0xFFFF_FFFF {
                    1
                } else {
                    length as usize
                };
                vars.push(HierVar {
                    full_path,
                    ty: fst_var_ty(tag).to_string(),
                    width,
                    range,
                    handle,
                });
            }
            _ => return Err(FstError(format!("unknown hierarchy tag {tag}"))),
        }
    }
    Ok(vars)
}

/// Decode one VCDATA_DYN_ALIAS2 block, adding per-handle toggle counts.
fn decode_vc_block(
    p: &[u8],
    handles: &[HierVar],
    window: Option<(f64, Option<f64>)>,
    timescale_s: f64,
    chain_counts: &mut [Vec<u64>],
) -> Result<(), FstError> {
    if p.len() < 24 {
        return Err(FstError("short vc block".into()));
    }
    // front: start(8) end(8) mem(8), varints bits_unc/bits_cmp/bits_count, bits_data,
    // varint waves_count, u8 packtype -> waves_data start.
    let mut o = 24usize;
    let (_bits_unc, no) = uvarint(p, o)?;
    o = no;
    let (bits_cmp, no) = uvarint(p, o)?;
    o = no;
    let (_bits_cnt, no) = uvarint(p, o)?;
    o = no;
    // THE FRAME IS EVERY SIGNAL'S VALUE AT THE START OF THIS BLOCK, and it is the baseline.
    //
    // A dump that writes `$dumpvars` before any `#time` — which is most of them — records those
    // initial values ONLY here. Skipping the frame left the first value-change record with
    // nothing to compare against, so it became the baseline and its transition was lost: one
    // toggle missing on every signal that ever changes, `clk` reading 9 where the same waveform
    // in VCD reads 10.
    //
    // Layout: ASCII, one byte per bit, `width` bytes per signal, in handle order. zlib-
    // compressed when the compressed and uncompressed lengths differ.
    let frame_start = o;
    let frame_end = (o + bits_cmp as usize).min(p.len());
    let frame: Vec<u8> = if bits_cmp == _bits_unc {
        p[frame_start..frame_end].to_vec()
    } else {
        zlib(&p[frame_start..frame_end]).unwrap_or_default()
    };
    o = frame_end;
    let (_waves_count, no) = uvarint(p, o)?;
    o = no;
    let _packtype = *p.get(o).ok_or_else(|| FstError("short vc block".into()))?;
    o += 1;
    let waves_start = o;

    // tail (from the block end backward): time_count, time_cmp, time_unc, time_data,
    // position_length, position_data.
    let n = p.len();
    let time_count = be_u64(p, n - 8)? as usize;
    let time_cmp = be_u64(p, n - 16)? as usize;
    let time_unc = be_u64(p, n - 24)? as usize;
    let time_data_end = n - 24;
    let time_data_start = time_data_end
        .checked_sub(time_cmp)
        .ok_or_else(|| FstError("vc time table underflow".into()))?;
    let pos_len_off = time_data_start
        .checked_sub(8)
        .ok_or_else(|| FstError("vc position underflow".into()))?;
    let pos_len = be_u64(p, pos_len_off)? as usize;
    let pos_end = pos_len_off;
    let pos_start = pos_end
        .checked_sub(pos_len)
        .ok_or_else(|| FstError("vc position underflow".into()))?;
    let waves_end = pos_start;
    let waves = &p[waves_start..waves_end];

    // optional time table (only needed for windowing): abs time per time-index.
    let times: Option<Vec<u64>> = match window {
        None => None,
        Some(_) => Some(decode_time_table(
            &p[time_data_start..time_data_end],
            time_cmp,
            time_unc,
            time_count,
        )?),
    };
    let (win_from, win_to) = window.unwrap_or((f64::NEG_INFINITY, None));

    // position/chain table: signed single-or-multi byte varints; enc = signed >> 1.
    // positive -> running byte offset into `waves` (value-1); negative -> alias handle.
    let chains = decode_position(&p[pos_start..pos_end], waves.len())?;

    // decode each physical chain's toggle counts, resolving aliases.
    let mut cache: HashMap<usize, ()> = HashMap::new(); // guard against alias cycles
    let widths = handle_widths(handles, chains.len());
    let reals = handle_reals(handles, chains.len());
    for (h, chain) in chains.iter().enumerate() {
        let target = resolve_alias(&chains, h, &mut cache)?;
        let off = match chains[target] {
            Chain::Data(off) => off,
            Chain::None => {
                merge_counts(&mut chain_counts[h], &[0]);
                continue;
            }
            Chain::Alias(_) => {
                continue;
            }
        };
        let _ = chain; // clarity
        let seg_end = next_data_offset(&chains, target).unwrap_or(waves.len());
        let counts = decode_chain(
            &waves[off..seg_end],
            widths[target].max(1),
            reals[target],
            frame_value(&frame, &widths, target),
            times.as_deref(),
            win_from,
            win_to,
            timescale_s,
        )?;
        merge_counts(&mut chain_counts[h], &counts);
    }
    Ok(())
}

/// Per-handle bit width, from the hierarchy (max width seen for that handle).
fn handle_widths(handles: &[HierVar], n: usize) -> Vec<usize> {
    let mut w = vec![0usize; n];
    for h in handles {
        if h.handle < n {
            w[h.handle] = w[h.handle].max(h.width);
        }
    }
    w
}

/// Per-handle "this handle carries a double" flag, matching `handle_widths`.
fn handle_reals(handles: &[HierVar], n: usize) -> Vec<bool> {
    let mut r = vec![false; n];
    for h in handles {
        if h.handle < n && h.ty == "real" {
            r[h.handle] = true;
        }
    }
    r
}

enum Chain {
    None,
    Data(usize),
    Alias(usize),
}

fn decode_position(data: &[u8], _waves_len: usize) -> Result<Vec<Chain>, FstError> {
    let mut chains = Vec::new();
    let mut acc: i64 = 0;
    let mut o = 0usize;
    // A zero entry means "the SAME alias as the previous entry", so the previous one has to be
    // carried. Read as a data offset instead — which is what a naive `>= 0` test does — it
    // points the signal at whatever chain came before, and that signal's own data is then
    // swallowed by its neighbour's segment. One net reports nothing, the next reports two
    // nets' worth.
    let mut prev_alias: i64 = 0;
    while o < data.len() {
        let start = o;
        let (raw, no) = uvarint(data, o)?;
        let nbytes = no - start;
        o = no;
        if raw & 1 == 0 {
            // a run of `raw >> 1` handles with no data of their own
            let run = raw >> 1;
            for _ in 0..run {
                chains.push(Chain::None);
            }
        } else {
            let enc = svalue_n(raw, nbytes) >> 1; // arithmetic shift keeps the sign
            match enc.cmp(&0) {
                std::cmp::Ordering::Greater => {
                    acc += enc;
                    chains.push(Chain::Data((acc - 1) as usize));
                }
                std::cmp::Ordering::Less => {
                    prev_alias = enc;
                    chains.push(Chain::Alias((-(enc + 1)) as usize));
                }
                std::cmp::Ordering::Equal => {
                    // repeat the previous alias
                    if prev_alias < 0 {
                        chains.push(Chain::Alias((-(prev_alias + 1)) as usize));
                    } else {
                        chains.push(Chain::None);
                    }
                }
            }
        }
    }
    Ok(chains)
}

fn resolve_alias(
    chains: &[Chain],
    mut h: usize,
    seen: &mut HashMap<usize, ()>,
) -> Result<usize, FstError> {
    seen.clear();
    loop {
        match chains.get(h) {
            Some(Chain::Alias(t)) => {
                if seen.insert(h, ()).is_some() {
                    return Err(FstError("alias cycle".into()));
                }
                h = *t;
            }
            Some(_) => return Ok(h),
            None => return Err(FstError("alias out of range".into())),
        }
    }
}

/// Byte offset in `waves` where the chain *after* the data-chain at index `from` starts.
fn next_data_offset(chains: &[Chain], from: usize) -> Option<usize> {
    chains
        .iter()
        .skip(from + 1)
        .filter_map(|c| match c {
            Chain::Data(o) => Some(*o),
            _ => None,
        })
        .min()
}

/// Decode one signal's value-change chain into toggle counts. Scalars → a single
/// count (value transitions). Vectors → per-bit Hamming counts (MSB..LSB).
/// One signal's value from the block frame: ASCII, `width` bytes, in handle order.
///
/// `None` when the frame is absent or too short — in which case the first change record becomes
/// the baseline and its transition is lost, which is the defect this exists to prevent.
fn frame_value<'a>(frame: &'a [u8], widths: &[usize], handle: usize) -> Option<&'a [u8]> {
    if frame.is_empty() {
        return None;
    }
    let mut at = 0usize;
    for (h, w) in widths.iter().enumerate() {
        // one byte per bit, and zero-length (variable-length) signals occupy none
        let w = *w;
        if h == handle {
            return (at + w <= frame.len()).then(|| &frame[at..at + w]);
        }
        at += w;
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn decode_chain(
    seg: &[u8],
    width: usize,
    is_real: bool,
    initial: Option<&[u8]>,
    times: Option<&[u64]>,
    win_from: f64,
    win_to: Option<f64>,
    timescale_s: f64,
) -> Result<Vec<u64>, FstError> {
    if seg.is_empty() {
        return Ok(if is_real || width <= 1 {
            vec![0]
        } else {
            vec![0; width]
        });
    }
    let (unclen, o) = uvarint(seg, 0)?;
    let data: Vec<u8> = if unclen == 0 {
        seg[o..].to_vec()
    } else {
        lz4::decompress(&seg[o..], unclen as usize).map_err(FstError)?
    };

    let in_window = |ti: usize| -> bool {
        match times {
            None => true,
            Some(t) => {
                let abs = *t.get(ti).unwrap_or(&0) as f64 * timescale_s;
                abs >= win_from
                    && match win_to {
                        Some(to) => abs < to,
                        None => true,
                    }
            }
        }
    };

    let mut p = 0usize;
    let mut time_index: usize = 0;
    if is_real {
        // A double: varint(tdelta<<1 | raw), then either the 8 raw bytes (raw=1) or — rarely —
        // a single byte standing for eight bits. Either way it is ONE value, so it counts as
        // one net, not eight. The frame holds the same 8 bytes, which is the baseline.
        // A NaN frame is "not yet driven" — the real's `x`, and the same rule applies: it is not
        // a level, so it does not become the baseline. A VCD has no equivalent (its first
        // `r` record IS the baseline), so seeding from it charges one extra toggle the moment
        // the signal is first driven. A NaN in a CHANGE record is a real value and does count,
        // which is what the VCD reader does with one.
        let mut prev: Option<[u8; 8]> = initial
            .and_then(|v| <[u8; 8]>::try_from(v).ok())
            .filter(|b| !f64::from_le_bytes(*b).is_nan());
        let mut count: u64 = 0;
        while p < data.len() {
            let (v, np) = uvarint(&data, p)?;
            p = np;
            let mut bytes = [0u8; 8];
            let take = if v & 1 == 1 { 8 } else { 1 };
            if p + take > data.len() {
                break;
            }
            bytes[..take].copy_from_slice(&data[p..p + take]);
            p += take;
            time_index += (v >> 1) as usize;
            if prev.map(|q| q != bytes).unwrap_or(false) && in_window(time_index) {
                count += 1;
            }
            prev = Some(bytes);
        }
        Ok(vec![count])
    } else if width <= 1 {
        // 1-bit: one varint per change; value transitions counted vs previous.
        // Seeded from the frame, so the FIRST change is a change and not the baseline.
        // Seeded from the frame, so the FIRST change record is a change and not the baseline.
        // An unknown initial value seeds nothing, exactly as an unknown change counts nothing.
        let mut prev: Option<char> = initial
            .and_then(|v| v.first())
            .and_then(|&b| level(b as char));
        let mut count: u64 = 0;
        while p < data.len() {
            let (v, np) = uvarint(&data, p)?;
            p = np;
            let (val, td) = if v & 1 == 0 {
                (if (v >> 1) & 1 == 1 { '1' } else { '0' }, (v >> 2) as usize)
            } else {
                (xz_char((v >> 1) & 7), (v >> 4) as usize)
            };
            time_index += td;
            // Same rule as the VCD reader: `x`/`z` is not a level. It counts no transition and
            // does not become the baseline — otherwise a signal whose FRAME value is `x`
            // (undriven at time zero, which is most of them) charges one extra toggle the
            // moment it is first driven.
            let Some(val) = level(val) else { continue };
            let changed = prev.map(|c| c != val).unwrap_or(false);
            if changed && in_window(time_index) {
                count += 1;
            }
            prev = Some(val);
        }
        Ok(vec![count])
    } else {
        // multi-bit: varint(tdelta<<1 | has_xz) then packed BE bytes (has_xz=0) or ASCII.
        let nbytes = width.div_ceil(8);
        let mut prev: Option<Vec<char>> = initial.filter(|v| v.len() == width).map(|v| {
            v.iter()
                .map(|&b| (b as char).to_ascii_lowercase())
                .collect()
        });
        let mut counts = vec![0u64; width];
        while p < data.len() {
            let (hdr, np) = uvarint(&data, p)?;
            p = np;
            let has_xz = hdr & 1 == 1;
            let td = (hdr >> 1) as usize;
            time_index += td;
            // A RECORD THAT RUNS PAST THE END OF THE BLOCK IS TRUNCATED, and the only safe
            // reading of it is none. Clamping the slice and then indexing as though it were
            // full length is what turned a short tail into `index out of bounds` — a library
            // panic, which a caller cannot catch and which takes the whole tool with it.
            // Found on a GTKWave-produced file, by the converter's own output.
            let need = if has_xz { width } else { nbytes };
            if p + need > data.len() {
                break;
            }
            let cur: Vec<char> = if has_xz {
                let s: Vec<char> = data[p..p + width].iter().map(|&b| b as char).collect();
                p += width;
                s
            } else {
                let raw = &data[p..p + nbytes];
                p += nbytes;
                // BITS ARE MSB-ALIGNED, NOT LSB-ALIGNED. FST packs a value starting at the
                // most significant bit of the first byte and pads at the END, so a 3-bit value
                // `011` is the byte `0110_0000` (0x60) — not `0000_0011`.
                //
                // Reading from the low end instead returned the padding: every signal whose
                // width is not a multiple of 8 decoded as all-zero and reported NO activity at
                // all. Widths that are a multiple of 8 are identical either way, which is why
                // every 8-, 16- and 32-bit bus looked correct and the defect survived a
                // cross-check against the same waveforms in VCD form.
                let mut bitv = Vec::with_capacity(width);
                for k in 0..width {
                    let byte = raw[k / 8];
                    bitv.push(if (byte >> (7 - (k % 8))) & 1 == 1 {
                        '1'
                    } else {
                        '0'
                    });
                }
                bitv
            };
            if let Some(prev) = &prev {
                if in_window(time_index) {
                    for (i, (a, b)) in cur.iter().zip(prev).enumerate() {
                        // per bit, an unknown is not a level — see the 1-bit path
                        let (Some(a), Some(b)) = (level(*a), level(*b)) else {
                            continue;
                        };
                        if a != b {
                            counts[i] += 1;
                        }
                    }
                }
            }
            // An unknown bit leaves the last KNOWN value in place, so `0 -> x -> 0` counts
            // nothing and `0 -> x -> 1` counts one. Replacing the whole vector would make the
            // unknown itself the baseline.
            prev = Some(match prev.take() {
                None => cur,
                Some(mut was) => {
                    for (i, c) in cur.iter().enumerate() {
                        if let (true, Some(c)) = (i < was.len(), level(*c)) {
                            was[i] = c;
                        }
                    }
                    was
                }
            });
        }
        Ok(counts)
    }
}

fn merge_counts(dst: &mut Vec<u64>, src: &[u64]) {
    if dst.len() < src.len() {
        dst.resize(src.len(), 0);
    }
    for (d, s) in dst.iter_mut().zip(src) {
        *d += *s;
    }
}

// ---- time table (zlib) ---------------------------------------------------------

fn decode_time_table(
    data: &[u8],
    cmp: usize,
    unc: usize,
    count: usize,
) -> Result<Vec<u64>, FstError> {
    let raw = if cmp == unc {
        data.to_vec()
    } else {
        zlib(data).map_err(FstError)?
    };
    let mut times = Vec::with_capacity(count);
    let mut o = 0usize;
    let mut acc: u64 = 0;
    while times.len() < count && o < raw.len() {
        let (d, no) = uvarint(&raw, o)?;
        o = no;
        acc += d;
        times.push(acc);
    }
    Ok(times)
}

// ---- small helpers -------------------------------------------------------------

fn be_u64(b: &[u8], o: usize) -> Result<u64, FstError> {
    b.get(o..o + 8)
        .map(|s| u64::from_be_bytes(s.try_into().unwrap()))
        .ok_or_else(|| FstError("truncated u64".into()))
}

fn uvarint(b: &[u8], mut o: usize) -> Result<(u64, usize), FstError> {
    let mut r: u64 = 0;
    let mut s = 0u32;
    loop {
        let x = *b.get(o).ok_or_else(|| FstError("varint overrun".into()))?;
        o += 1;
        r |= ((x & 0x7f) as u64) << s;
        if x & 0x80 == 0 {
            return Ok((r, o));
        }
        s += 7;
    }
}

/// Interpret an unsigned-decoded LEB128 as a signed value, sign-extending from the top bit of
/// the highest 7-bit group the ENCODING occupied.
///
/// `nbytes` is how many bytes the varint actually took, and it has to be passed in: inferring
/// it from the magnitude of the decoded value is wrong whenever the value is small enough to
/// fit in fewer groups than were written. `f9 00` encodes 121 in two bytes, so its sign bit is
/// bit 13 and the value is +121; read as one group, bit 6 is set and it becomes -7.
///
/// That mattered: in the chain table a negative value means "alias", so a 2-byte positive
/// offset was read as an alias to another handle. The signal took its width from the wrong
/// signal, and its own data was swallowed by the previous chain — one net reporting nothing and
/// its neighbour counting two nets' worth. Only for values that need a second byte, which is
/// why it appeared on 7 of 200 generated dumps and not on the small ones.
fn svalue_n(raw: u64, nbytes: usize) -> i64 {
    if raw == 0 {
        return 0;
    }
    let group_bits = (nbytes.max(1) * 7) as u32;
    if group_bits >= 64 {
        return raw as i64;
    }
    let sign_bit = 1u64 << (group_bits - 1);
    if raw & sign_bit != 0 {
        (raw | !((1u64 << group_bits) - 1)) as i64
    } else {
        raw as i64
    }
}

fn cstr(b: &[u8], p: &mut usize) -> String {
    let start = *p;
    while *p < b.len() && b[*p] != 0 {
        *p += 1;
    }
    let s = String::from_utf8_lossy(&b[start..*p]).into_owned();
    if *p < b.len() {
        *p += 1; // skip NUL
    }
    s
}

fn xz_char(code: u64) -> char {
    match code {
        0 => 'x',
        1 => 'z',
        2 => 'h',
        3 => 'u',
        4 => 'w',
        5 => 'l',
        6 => '-',
        _ => '?',
    }
}

/// Real-typed variables carry a double, not a bit vector, so they stay one net instead of
/// expanding per bit.
///
/// The tag numbers are `fstVarType`: REAL=3, REAL_PARAMETER=4, REALTIME=20, SV_SHORTREAL=29.
/// An earlier list here read 2, 6, 7, 18 — PARAMETER, SUPPLY0, SUPPLY1 and PORT — which named
/// every real `Freq[2]` after one of its mantissa bytes while scalarising three types that
/// really are bit vectors.
fn fst_var_ty(tag: u8) -> &'static str {
    match tag {
        3 | 4 | 20 | 29 => "real",
        // FST_VT_GEN_STRING: a variable-length payload, not a value. It has no bits to toggle,
        // it occupies NO space in the frame, and the VCD reader already ignores the `s` records
        // its dumps write — so counting it here made a VHDL string report activity that its own
        // VCD did not.
        21 => "string",
        _ => "wire",
    }
}

fn zlib(data: &[u8]) -> Result<Vec<u8>, String> {
    miniz_oxide::inflate::decompress_to_vec_zlib(data).map_err(|e| format!("zlib: {e:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Vec<u8> {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/counter.fst");
        std::fs::read(path).expect("counter.fst fixture")
    }

    /// A position-table entry that needed two bytes must sign-extend from bit 13, not bit 6.
    ///
    /// These are the seven bytes from the block that exposed it. The last entry is `f9 00`, a
    /// two-byte LEB128 for 121 — positive, so a data offset. Sign-extending from the top of the
    /// highest group the VALUE needs (bit 6, which is set in 121) makes it -7, and a negative
    /// entry means "alias": the signal took its width from an unrelated handle and its own
    /// segment was swallowed by the chain before it.
    #[test]
    fn a_two_byte_position_entry_is_an_offset_not_an_alias() {
        let chains = decode_position(&[0x03, 0x29, 0x27, 0x35, 0x2f, 0xf9, 0x00], 206).unwrap();
        let offs: Vec<Option<usize>> = chains
            .iter()
            .map(|c| match c {
                Chain::Data(o) => Some(*o),
                _ => None,
            })
            .collect();
        assert_eq!(
            offs,
            vec![Some(0), Some(20), Some(39), Some(65), Some(88), Some(148)]
        );
    }

    /// A zero chain entry repeats the PREVIOUS alias; it is not an offset of zero.
    #[test]
    fn a_zero_position_entry_repeats_the_previous_alias() {
        // 0x03 -> data at 0; 0x7d -> signed -3, i.e. alias to handle 1; 0x01 -> repeat it
        let chains = decode_position(&[0x03, 0x7d, 0x01], 64).unwrap();
        assert!(matches!(chains[0], Chain::Data(0)));
        assert!(matches!(chains[1], Chain::Alias(1)));
        assert!(
            matches!(chains[2], Chain::Alias(1)),
            "a repeat, not a second data chain"
        );
    }

    /// A scalar written in nine-state codes: `H` and `L` are levels, `W` is not.
    ///
    /// Straight at `decode_chain`, because the conversion path drops these — the codes are
    /// `FST_RCV_*`, `1 | (state << 1) | (tdelta << 4)`, with the state indexing "xzhuwl-?".
    #[test]
    fn nine_state_scalar_levels_are_weak_ones_and_zeros() {
        // u (frame) -> H -> L -> W -> H, one time step apart
        let seg = [0x00, 0x15, 0x1b, 0x19, 0x15];
        let counts = decode_chain(
            &seg,
            1,
            false,
            Some(b"u"),
            None,
            f64::NEG_INFINITY,
            None,
            1e-9,
        )
        .unwrap();
        // the `u` seeds nothing, H->L is a transition, the W is skipped without becoming the
        // baseline, so the following H is a transition from the L
        assert_eq!(counts, vec![2]);
    }

    /// The same waveform in FST form must produce the same counts as `ninestate.vcd`, whose
    /// numbers are derived by hand in `tests/vcd_robustness.rs`.
    ///
    /// It covers, in one file, everything the two readers used to disagree about: nine-state
    /// levels, a real that must stay one net (and whose NaN frame is "not yet driven", not a
    /// baseline), and a string that carries no activity and occupies no space in the frame.
    #[test]
    fn nine_state_and_real_agree_with_the_vcd_of_the_same_run() {
        let bytes = std::fs::read(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/ninestate.fst"
        ))
        .expect("ninestate.fst");
        let f = Fst::parse(&bytes, None).unwrap();
        let got = |n: &str| f.idx.toggles.get(n).copied().unwrap_or(0);
        assert_eq!(got("top.clk"), 4);
        // `top.weak` is deliberately not asserted here: vcd2fst discards nine-state values on a
        // SCALAR (GTKWave's own fst2vcd reads the same file back with no changes for it), so the
        // fixture cannot carry them. The vector below does round-trip them, and the scalar path
        // is covered directly by `nine_state_scalar_levels_are_weak_ones_and_zeros`.
        assert_eq!(got("top.bus[3]"), 1);
        assert_eq!(got("top.bus[2]"), 2);
        assert_eq!(got("top.bus[1]"), 0);
        assert_eq!(got("top.bus[0]"), 1);
        assert_eq!(
            got("top.freq"),
            2,
            "one net, and the NaN frame is not its baseline"
        );
        assert!(!f.idx.toggles.keys().any(|k| k.starts_with("top.freq[")));
        assert_eq!(got("top.label"), 0, "a string has no switching activity");
    }

    #[test]
    fn counter_fst_matches_ground_truth() {
        let f = Fst::parse(&fixture(), None)
            .unwrap()
            .with_scope(Some("counter_tb.dut".into()));
        // timescale 1ps, end 395000 -> 3.95e-7 s
        assert!((f.sim_time_s - 395_000.0 * 1e-12).abs() < 1e-15);
        let rate = |net: &str| f.toggle_rate(net);
        // clock: 79 transitions / 395 ns = 2.0e8; q1: 13; d/q0/y: 14
        assert!((rate("clk_in") - 79.0 / f.sim_time_s).abs() < 1.0);
        assert!((rate("clk_in") - 2.0e8).abs() < 1.0);
        assert!((rate("q1") - 13.0 / f.sim_time_s).abs() < 1.0);
        assert!((rate("d") - 14.0 / f.sim_time_s).abs() < 1.0);
        assert!((rate("q0") - 14.0 / f.sim_time_s).abs() < 1.0);
        // aliased names across scopes are detected
        assert!(f.collisions() > 0);
    }

    #[test]
    fn counter_fst_bus_per_bit_totals_78() {
        let f = Fst::parse(&fixture(), None).unwrap();
        // per-bit cascade of a 32-bit incrementing counter: 40,20,10,5,2,1 = 78
        let sum: u64 = f
            .idx
            .toggles
            .iter()
            .filter(|(k, _)| k.starts_with("counter_tb.i["))
            .map(|(_, &v)| v)
            .sum();
        assert_eq!(sum, 78, "32-bit bus per-bit Hamming total");
        // LSB toggles every increment
        assert_eq!(f.idx.toggles.get("counter_tb.i[0]").copied(), Some(40));
    }

    #[test]
    fn fst_equals_vcd_on_matched_pair() {
        // The multiset of nonzero toggle counts must match the matched VCD dump
        // (same run). Compare per-underlying-signal counts (dedup by value+path set).
        let fst = Fst::parse(&fixture(), None).unwrap();
        let vpath = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/counter.vcd");
        let vcd = crate::vcd::Vcd::load(vpath).unwrap();
        let mut fset: Vec<u64> = fst
            .idx
            .toggles
            .values()
            .copied()
            .filter(|&v| v > 0)
            .collect();
        let mut vset: Vec<u64> = vcd
            .idx
            .toggles
            .values()
            .copied()
            .filter(|&v| v > 0)
            .collect();
        fset.sort_unstable();
        vset.sort_unstable();
        // FST attributes each chain to every aliased name; VCD (last-wins per symbol)
        // to one. Compare the *distinct* count values rather than raw multiplicity.
        fset.dedup();
        vset.dedup();
        assert_eq!(fset, vset, "FST vs VCD distinct toggle counts");
    }
}
