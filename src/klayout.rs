//! KLayout net-dump reader — parses the neutral intermediate a headless-KLayout
//! driver emits (`LayoutToNetlist` connectivity + per-layer metal geometry) into
//! loom's [`Spef`](crate::spef::Spef) RC model plus the
//! [`EmGeom`](crate::emgeom::EmGeom) sidecar. Keeping the parser here (not in the
//! engine) means the KLayout coupling is a *data* boundary: the driver is a thin
//! dumper, and SPEF serialization stays in loom.
//!
//! Neutral format — line-oriented, `#` comments; a `NET` opens a block that owns
//! the following lines until the next `NET`:
//! ```text
//! # vyges-klayout-netdump v1
//! DESIGN <top>
//! NET  <net> <total_cap_ff>
//! PIN  <inst> <pin> <dir>                 # dir: I|O|B (advisory; unused today)
//! SEG  <a> <b> <ohm> <layer> <w_um> <l_um># one resistive metal segment
//! GCAP <node> <ff>                        # grounded cap
//! CCAP <other_net> <ff>                   # coupling cap to another net
//! ```
//! Node convention the driver uses (so [`Spef::to_spef`] maps cleanly): the net
//! node is the net name, a per-layer internal node is `<net>^<layer>`, a pin node
//! is `<inst>:<pin>`.
//!
//! Pure std — robust to malformed lines (skips them) — fully unit-tested offline.

use crate::emgeom::{EmGeom, SegGeom};
use crate::spef::{NetRc, Spef};
use std::collections::BTreeMap;

/// Parse a net-dump into `(Spef, EmGeom)`. Never panics: unknown keywords and
/// malformed lines are skipped; numbers that fail to parse default the entry out.
pub fn parse(text: &str) -> (Spef, EmGeom) {
    let mut nets: BTreeMap<String, NetRc> = BTreeMap::new();
    let mut geom = EmGeom::default();
    let mut cur: Option<String> = None;

    for raw in text.lines() {
        let t = raw.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        let k: Vec<&str> = t.split_whitespace().collect();
        match k.first().copied() {
            Some("DESIGN") => {
                if let Some(d) = k.get(1) {
                    geom.design = d.to_string();
                }
            }
            Some("NET") if k.len() >= 2 => {
                let name = k[1].to_string();
                let cap = k.get(2).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                nets.entry(name.clone()).or_insert_with(|| NetRc {
                    net_node: name.clone(),
                    ..Default::default()
                });
                if let Some(rc) = nets.get_mut(&name) {
                    rc.cap_ff = cap;
                }
                cur = Some(name);
            }
            Some("PIN") if k.len() >= 3 => {
                if let Some(rc) = cur.as_ref().and_then(|n| nets.get_mut(n)) {
                    let (inst, pin) = (k[1].to_string(), k[2].to_string());
                    let node = format!("{inst}:{pin}");
                    rc.pins.push((inst, pin, node));
                }
            }
            Some("SEG") if k.len() >= 7 => {
                let net = match &cur {
                    Some(n) => n.clone(),
                    None => continue,
                };
                let ohm = k[3].parse::<f64>();
                let w = k[5].parse::<f64>();
                let l = k[6].parse::<f64>();
                if let (Ok(ohm), Ok(width_um), Ok(length_um)) = (ohm, w, l) {
                    let (a, b, layer) = (k[1].to_string(), k[2].to_string(), k[4].to_string());
                    if let Some(rc) = nets.get_mut(&net) {
                        rc.res_ohm += ohm;
                        rc.res.push((a.clone(), b.clone(), ohm));
                    }
                    geom.segs.push(SegGeom {
                        net,
                        a,
                        b,
                        layer,
                        width_um,
                        length_um,
                        res_ohm: ohm,
                    });
                }
            }
            Some("GCAP") if k.len() >= 3 => {
                if let (Some(rc), Ok(c)) =
                    (cur.as_ref().and_then(|n| nets.get_mut(n)), k[2].parse::<f64>())
                {
                    rc.ground.push((k[1].to_string(), c));
                }
            }
            Some("CCAP") if k.len() >= 3 => {
                if let (Some(name), Ok(c)) = (cur.clone(), k[2].parse::<f64>()) {
                    let other = k[1].to_string();
                    if let Some(rc) = nets.get_mut(&name) {
                        rc.coupling_ff += c;
                        rc.coupling.push((other, c));
                    }
                }
            }
            _ => {}
        }
    }

    (Spef { nets }, geom)
}

/// Parse from a file path.
pub fn load(path: &str) -> std::io::Result<(Spef, EmGeom)> {
    let text = std::fs::read_to_string(path)?;
    Ok(parse(&text))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DUMP: &str = "\
# vyges-klayout-netdump v1
DESIGN blk
NET clk 14.0
PIN u1 Y O
PIN u2 A I
SEG clk clk^met1 350 met1 0.14 12.0
SEG clk^met1 u2:A 40 met2 0.2 4.0
GCAP clk 12.0
CCAP dat 2.0
NET dat 5.0
PIN u3 Q O
SEG dat u4:D 80 met1 0.14 3.0
CCAP clk 2.0
";

    #[test]
    fn builds_spef_and_geom() {
        let (spef, geom) = parse(DUMP);
        assert_eq!(spef.nets.len(), 2);
        let clk = spef.nets.get("clk").unwrap();
        assert_eq!(clk.cap_ff, 14.0);
        assert_eq!(clk.res_ohm, 390.0); // 350 + 40
        assert!(clk.pins.iter().any(|(i, p, _)| i == "u2" && p == "A"));
        assert_eq!(clk.coupling_ff, 2.0);
        assert_eq!(clk.ground, vec![("clk".to_string(), 12.0)]);

        assert_eq!(geom.design, "blk");
        assert_eq!(geom.segs.len(), 3);
        let m1 = geom.segs.iter().find(|s| s.layer == "met1" && s.net == "clk").unwrap();
        assert_eq!(m1.width_um, 0.14);
        assert_eq!(m1.length_um, 12.0);
        assert_eq!(m1.res_ohm, 350.0);
    }

    #[test]
    fn writes_valid_spef() {
        // the built Spef must serialize + re-parse (writer/reader closure)
        let (spef, _) = parse(DUMP);
        let text = spef.to_spef(&crate::spef::WriteOpts::default());
        let back = Spef::parse(&text);
        assert_eq!(back.nets.get("clk").unwrap().res_ohm, 390.0);
        assert!(back.nets.get("clk").unwrap().pins.iter().any(|(i, p, _)| i == "u2" && p == "A"));
    }
}
