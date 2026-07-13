//! EM geometry sidecar — the per-segment (layer, width, length) that a
//! standard SPEF does **not** carry but electromigration sign-off requires.
//!
//! Current-density screening needs `J = I / (width · thickness)` (or the LEF
//! `DCCURRENTDENSITY` form `limit = J_layer · width`), yet a SPEF `*RES` is only
//! `(node_a, node_b, ohm)` — no metal layer, no width. Rather than bend the SPEF
//! (which would break portability to STA / other tools), the extractor writes a
//! companion sidecar keyed to the same net + node names. Any RCX source
//! (KLayout, OpenRCX, magic, commercial) can emit it; the EM engine reads it.
//!
//! Format — line-oriented, `#` comments, one `SEG` per resistive metal segment:
//! ```text
//! # vyges-em-geom v1
//! DESIGN blk
//! SEG <net> <node_a> <node_b> <layer> <width_um> <length_um> <res_ohm>
//! ```
//! Pure std — fully unit-tested offline.

use std::collections::BTreeMap;

/// One resistive metal segment, cross-referenced to a SPEF `*RES` by
/// `(net, a, b)`. `width_um`/`length_um` are the physical metal dimensions the
/// EM limit is computed from; `res_ohm` mirrors the SPEF value (convenience so
/// the EM engine need not join back for the resistance).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SegGeom {
    pub net: String,
    pub a: String,
    pub b: String,
    pub layer: String,
    pub width_um: f64,
    pub length_um: f64,
    pub res_ohm: f64,
}

#[derive(Debug, Clone, Default)]
pub struct EmGeom {
    pub design: String,
    pub segs: Vec<SegGeom>,
}

#[derive(Debug)]
pub struct EmGeomError(pub String);
impl std::fmt::Display for EmGeomError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "em-geom error: {}", self.0)
    }
}
impl std::error::Error for EmGeomError {}

impl EmGeom {
    /// Serialize to the sidecar text format. Deterministic (segments in the
    /// order supplied by the caller — the extractor emits net/segment order).
    pub fn to_text(&self) -> String {
        let mut s = String::from("# vyges-em-geom v1\n");
        if !self.design.is_empty() {
            s.push_str(&format!("DESIGN {}\n", self.design));
        }
        for g in &self.segs {
            s.push_str(&format!(
                "SEG {} {} {} {} {} {} {}\n",
                g.net,
                g.a,
                g.b,
                g.layer,
                trim(g.width_um),
                trim(g.length_um),
                trim(g.res_ohm),
            ));
        }
        s
    }

    /// Parse the sidecar text. Unknown keywords and malformed `SEG` lines are
    /// skipped (a reader must never crash on a slightly-off input — the suite's
    /// robustness contract); a `SEG` needs all 7 fields with numeric tail.
    pub fn parse(text: &str) -> EmGeom {
        let mut out = EmGeom::default();
        for raw in text.lines() {
            let t = raw.trim();
            if t.is_empty() || t.starts_with('#') {
                continue;
            }
            let toks: Vec<&str> = t.split_whitespace().collect();
            match toks.first().copied() {
                Some("DESIGN") => {
                    if let Some(d) = toks.get(1) {
                        out.design = d.to_string();
                    }
                }
                Some("SEG") if toks.len() >= 8 => {
                    let (w, l, r) = (
                        toks[5].parse::<f64>(),
                        toks[6].parse::<f64>(),
                        toks[7].parse::<f64>(),
                    );
                    if let (Ok(width_um), Ok(length_um), Ok(res_ohm)) = (w, l, r) {
                        out.segs.push(SegGeom {
                            net: toks[1].to_string(),
                            a: toks[2].to_string(),
                            b: toks[3].to_string(),
                            layer: toks[4].to_string(),
                            width_um,
                            length_um,
                            res_ohm,
                        });
                    }
                }
                _ => {}
            }
        }
        out
    }

    pub fn load(path: &str) -> Result<EmGeom, EmGeomError> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| EmGeomError(format!("{path}: {e}")))?;
        Ok(EmGeom::parse(&text))
    }

    /// Group segments by net (for per-net EM reporting).
    pub fn by_net(&self) -> BTreeMap<&str, Vec<&SegGeom>> {
        let mut m: BTreeMap<&str, Vec<&SegGeom>> = BTreeMap::new();
        for g in &self.segs {
            m.entry(g.net.as_str()).or_default().push(g);
        }
        m
    }
}

fn trim(v: f64) -> String {
    if v == 0.0 {
        return "0".into();
    }
    if v.fract() == 0.0 && v.abs() < 1e15 {
        return format!("{}", v as i64);
    }
    let s = format!("{v:.6}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let g = EmGeom {
            design: "blk".into(),
            segs: vec![
                SegGeom {
                    net: "clk".into(),
                    a: "clk".into(),
                    b: "clk^met1".into(),
                    layer: "met1".into(),
                    width_um: 0.14,
                    length_um: 12.0,
                    res_ohm: 350.0,
                },
                SegGeom {
                    net: "clk".into(),
                    a: "clk^met1".into(),
                    b: "u2:A".into(),
                    layer: "met2".into(),
                    width_um: 0.2,
                    length_um: 4.0,
                    res_ohm: 40.0,
                },
            ],
        };
        let back = EmGeom::parse(&g.to_text());
        assert_eq!(back.design, "blk");
        assert_eq!(back.segs.len(), 2);
        assert_eq!(back.segs[0], g.segs[0]);
        assert_eq!(back.by_net()["clk"].len(), 2);
    }

    #[test]
    fn skips_garbage() {
        let g = EmGeom::parse("# hdr\nJUNK a b\nSEG n a b met1 0.1 1\nSEG n a b met1 0.1 1 2\n");
        assert_eq!(g.segs.len(), 1); // short SEG dropped, good SEG kept
        assert_eq!(g.segs[0].layer, "met1");
    }
}
