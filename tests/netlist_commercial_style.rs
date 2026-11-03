//! A whole-hierarchy netlist in commercial-writer style (`tests/data/netlist/
//! commercial_style_hier.v`, a synthetic fixture written for loom): header comment block, every
//! module in one file, no scalar wire declarations, constant ties through `assign`, an inout pad,
//! and library datapath modules nothing instantiates — read plain and gzipped.
//!
//! It pins three behaviours together: the top is chosen right (by the header's block name, with
//! orphan modules beside it), the hierarchy flattens to its cells, and `.v.gz` reads by default.

use std::collections::HashSet;
use vyges_loom::netlist::{self, FlattenOptions, Inst, Netlist};

const FIXTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/netlist/commercial_style_hier.v");

fn inst<'a>(nl: &'a Netlist, name: &str) -> &'a Inst {
    nl.insts.iter().find(|x| x.name == name).unwrap_or_else(|| panic!("no instance {name}"))
}

fn conns(x: &Inst) -> Vec<(&str, &str)> {
    x.conns.iter().map(|(p, n)| (p.as_str(), n.as_str())).collect()
}

fn library() -> HashSet<String> {
    ["XOR2_X1", "XNOR2_X1", "BIDIR_PAD", "NAND2_X1", "MUX2_X1", "DFF_X1", "INV_X1", "BUF_X1", "AND2_X1"]
        .iter()
        .map(|s| s.to_string())
        .collect()
}

/// Three modules are instantiated by nothing (the top and two datapath modules); the header's
/// block name picks the top, and without it the read is refused with the candidates listed.
#[test]
fn the_top_is_the_block_the_header_names() {
    let nl = netlist::load(FIXTURE).unwrap();
    assert_eq!(nl.module, "chip_top");
    assert_eq!(nl.modules.len(), 5);
    let text = std::fs::read_to_string(FIXTURE).unwrap();
    let headless: String = text.lines().filter(|l| !l.contains("Block Name")).collect::<Vec<_>>().join("\n");
    let err = netlist::parse(&headless).unwrap_err().0;
    assert!(err.contains("3 modules are instantiated by no other module"), "{err}");
    assert!(err.contains("chip_top, dp_add_w2, dp_sub_w4"), "{err}");
    let named = netlist::parse_all_top(&headless, "chip_top").unwrap();
    assert_eq!(named.top().module, "chip_top");
    // the top alone holds 7 instances, two of them modules — and its health says to flatten
    assert_eq!(nl.insts.len(), 7);
    assert!(nl.health().unwrap().contains("flatten"));
}

/// Flattened: every cell under its path, child nets named by the parent's net they join, the
/// pad's inout kept apart, and the `assign`/pin constants as ties.
#[test]
fn the_hierarchy_flattens_to_its_cells() {
    let opts = FlattenOptions { library_cells: Some(library()), ..Default::default() };
    let (nl, rep) = netlist::load_flat(FIXTURE, &opts).unwrap();
    let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
    assert_eq!(
        names,
        ["u_core/U1", "u_core/U2", "u_core/U3", "u_core/y_reg_0_", "u_core/y_reg_1_", "U10", "U11", "U12", "u_pad/U0", "pad_reg", "U13"]
    );
    assert_eq!(conns(inst(&nl, "u_core/U1")), [("A1", "a[0]"), ("A2", "b[0]"), ("ZN", "u_core/n1")]);
    assert_eq!(conns(inst(&nl, "u_core/U3")), [("I0", "u_core/n1"), ("I1", "u_core/n2"), ("S", "n_sel"), ("Z", "u_core/n3")]);
    assert_eq!(conns(inst(&nl, "u_core/y_reg_1_")), [("D", "u_core/n1"), ("CK", "clk"), ("Q", "y_int[1]")]);
    let pad = inst(&nl, "u_pad/U0");
    assert_eq!(conns(pad), [("PAD", "pad_io"), ("I", "y_int[0]"), ("C", "pad_in")]);
    assert_eq!(pad.ties, [("OEN".to_string(), false)]);
    let and = inst(&nl, "U13");
    assert_eq!(conns(and), [("A1", "n_pq"), ("Z", "spare")]);
    assert_eq!(and.ties, [("A2".to_string(), false)], "the assign tie reaches the pin");
    assert_eq!(nl.inouts, ["pad_io"]);
    assert_eq!(nl.outputs, ["y[1]", "y[0]", "spare"]);
    assert_eq!((rep.expanded, rep.leaves), (2, 11));
    assert!(rep.undefined_cells.is_empty(), "{:?}", rep.undefined_cells);
    assert_eq!(rep.health(), None);
}

/// The same file gzipped (written here) reads the same — by default, and through
/// `Design::load`, which strips the `.gz` before choosing a reader.
#[cfg(feature = "gzip")]
#[test]
fn the_gzipped_copy_reads_the_same() {
    let text = std::fs::read(FIXTURE).unwrap();
    let mut gz = vec![0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 0, 3];
    gz.extend(miniz_oxide::deflate::compress_to_vec(&text, 6));
    gz.extend_from_slice(&[0; 4]); // CRC-32 (not checked)
    gz.extend_from_slice(&(text.len() as u32).to_le_bytes());
    let dir = std::env::temp_dir().join(format!("loom-commercial-gz-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let f = dir.join("commercial_style_hier.v.gz");
    std::fs::write(&f, gz).unwrap();
    let f = f.to_str().unwrap();
    let key = |nl: &Netlist| nl.insts.iter().map(|x| (x.name.clone(), x.conns.clone(), x.ties.clone())).collect::<Vec<_>>();
    let opts = FlattenOptions::default();
    let (plain, _) = netlist::load_flat(FIXTURE, &opts).unwrap();
    let (zipped, _) = netlist::load_flat(f, &opts).unwrap();
    assert_eq!(key(&zipped), key(&plain));
    let mut d = vyges_loom::Design::new();
    assert_eq!(d.load(f).unwrap(), "netlist");
    let rep = d.flatten_netlist(&[]).unwrap();
    assert_eq!(rep.leaves, 11);
    assert_eq!(key(d.netlist.as_ref().unwrap()), key(&plain));
    std::fs::remove_dir_all(&dir).ok();
}
