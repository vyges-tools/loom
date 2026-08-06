//! Schema-driven robustness for the SPICE reader.
//!
//! Same method as `lef_robustness.rs` and `def_robustness.rs`, and for the same reason: this
//! reader models a deliberately narrow slice of SPICE — `M Q R C L D X`, enough for LVS
//! connectivity and device recognition — and everything else must be *ignored*. Ignoring is not
//! the same as tolerating. A construct we do not model can still change the ones we do, by
//! consuming a line we needed, by being mistaken for a device, or by ending a `.subckt` early.
//!
//! `tests/data/spice_elements.txt` is the vocabulary: every element letter a netlist may carry
//! and the dot-commands that structure it, from the Berkeley SPICE3 element set and ngspice's
//! manual. The assertion is that injecting any of them leaves the devices we extract unchanged.
//!
//! **Why a sweep rather than a corpus.** Running over ngspice's own 114 test netlists passes
//! trivially: they are analog test circuits built almost entirely from `V`, `I`, `E`–`H` and `B`
//! sources, so the reader extracts 298 devices out of them and an invariant about *extracted*
//! devices says nothing about the hundreds of lines it walked past. The corpus proves we do not
//! crash; only the sweep proves we do not misread.

use std::collections::BTreeMap;

use vyges_loom::spice::{Device, Netlist};

fn vocabulary() -> (Vec<char>, Vec<String>) {
    let mut elements = Vec::new();
    let mut dots = Vec::new();
    for line in include_str!("data/spice_elements.txt").lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if let Some(d) = line.strip_prefix('.') {
            dots.push(format!(".{d}"));
        } else if let Some(c) = line.chars().next() {
            elements.push(c);
        }
    }
    (elements, dots)
}

/// The letters this reader is *supposed* to act on. Injecting one of these may legitimately add
/// a device, so they are asserted individually rather than swept.
const MODELLED: &[char] = &['M', 'Q', 'R', 'C', 'L', 'D', 'X'];

/// Structural dot-commands that legitimately change the parse — they open, close or name the
/// scope the devices live in.
///
/// `.control` is here because it opens a block whose contents are commands rather than circuit;
/// injecting one without its `.endc` legitimately swallows what follows, which is what a reader
/// should do with an unterminated block. Its *paired* form is asserted separately, in
/// `a_control_block_is_not_circuit`.
const STRUCTURAL: &[&str] = &[
    ".subckt", ".ends", ".global", ".include", ".lib", ".end", ".title", ".control", ".endc",
];

/// A netlist exercising everything this reader claims to read.
fn good(extra: &str) -> String {
    format!(
        ".title reference\n\
         .global vdd vss\n\
         .subckt inv a y vdd vss\n\
         {extra}\
         M1 y a vdd vdd pfet w=1u l=0.15u\n\
         M2 y a vss vss nfet w=0.5u l=0.15u\n\
         C1 y vss 1f\n\
         .ends\n\
         X1 in out vdd vss inv\n\
         R1 out vss 10k\n\
         .end\n"
    )
}

/// Everything the reader extracted, as one comparable value.
fn fingerprint(nl: &Netlist) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    let one = |d: &Device| {
        format!("{} {} {:?} params={:?}", d.kind, d.model, d.nodes, d.params)
    };
    for d in &nl.devices {
        m.insert(format!("{}:{}", d.kind, d.name), one(d));
    }
    m
}

fn parse(text: &str) -> Netlist {
    Netlist::parse(text, None).expect("the reference netlist is valid SPICE")
}

/// The baseline: if this is wrong every other case is meaningless.
#[test]
fn the_known_good_netlist_reads_as_expected() {
    let nl = parse(&good(""));
    let f = fingerprint(&nl);
    // flattened: two MOSFETs and a cap from the inv instance, plus the top-level R
    assert_eq!(nl.devices.len(), 4, "{:?}", f.keys().collect::<Vec<_>>());
    assert_eq!(nl.devices.iter().filter(|d| d.kind == 'M').count(), 2);
    assert_eq!(nl.devices.iter().filter(|d| d.kind == 'C').count(), 1);
    assert_eq!(nl.devices.iter().filter(|d| d.kind == 'R').count(), 1);
    let m = nl.devices.iter().find(|d| d.kind == 'M').expect("a mosfet");
    assert_eq!(m.nodes.len(), 4, "a MOSFET has four terminals");
    assert!(m.params.contains_key("w") && m.params.contains_key("l"), "w/l are read");
}

/// **The generated sweep.** An element letter this reader does not model must not change the
/// devices it does — not by being mistaken for one, and not by consuming the line after it.
#[test]
fn no_unmodelled_element_can_perturb_the_devices_we_read() {
    let (elements, _) = vocabulary();
    let base = fingerprint(&parse(&good("")));
    let mut broke = Vec::new();
    for e in elements {
        if MODELLED.contains(&e) {
            continue;
        }
        // the forms these letters take: a plain two-terminal, a controlled source (four nodes),
        // and one carrying a model name and parameters
        for body in [
            format!("{e}zz n1 n2 1.0\n"),
            format!("{e}zz n1 n2 n3 n4 2.5\n"),
            format!("{e}zz n1 n2 somemodel w=1u l=2u\n"),
        ] {
            let got = fingerprint(&parse(&good(&body)));
            if got != base {
                broke.push(format!("{e}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} element letter(s) changed what we read:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// The same for dot-commands. One that is not structural must leave the devices alone —
/// including the analysis and control blocks, which carry arbitrary text.
#[test]
fn no_unmodelled_dot_command_can_perturb_the_devices_we_read() {
    let (_, dots) = vocabulary();
    let base = fingerprint(&parse(&good("")));
    let mut broke = Vec::new();
    for d in dots {
        if STRUCTURAL.contains(&d.as_str()) {
            continue;
        }
        for body in [
            format!("{d}\n"),
            format!("{d} 1 2 3\n"),
            format!("{d} nfet nmos level=54 version=4.5\n"),
        ] {
            let got = fingerprint(&parse(&good(&body)));
            if got != base {
                broke.push(format!("{d}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} dot-command(s) changed what we read:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// A `.control` … `.endc` block contains *simulator commands*, not netlist — arbitrary text that
/// can look like anything, including a device line. Read as circuit it invents devices that the
/// design does not have, which an LVS compare then reports as a mismatch against the layout.
#[test]
fn a_control_block_is_not_circuit() {
    let base = fingerprint(&parse(&good("")));
    let with = fingerprint(&parse(&good(
        ".control\nrun\nM99 a b c d fake\nR99 x y 1k\nprint all\n.endc\n",
    )));
    assert_eq!(with, base, "a .control block must not contribute devices");
}
