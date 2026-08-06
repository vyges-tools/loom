//! Grammar conformance for the SPEF reader — the constructs a real writer emits.
//!
//! Every case here was a **silent** defect before it was a test. `Spef::parse` cannot fail, so
//! a construct it does not understand does not raise: it returns a smaller answer. Corner
//! triplets read as zero parasitics on every net; a lower-case file read as no nets at all;
//! an escaped hierarchical name that matches nothing in the design. None of that shows up as
//! an error, and downstream it looks like ideal interconnect.
//!
//! Written after two coupling bugs in the same reader were traced to exactly this: nothing
//! exercised the grammar, only the happy path our own writer emits.

use vyges_loom::spef::Spef;

fn hdr(cu: &str, ru: &str) -> String {
    format!(
        "*SPEF \"IEEE 1481-1999\"\n*DESIGN \"b\"\n*DATE \"x\"\n*DIVIDER /\n*DELIMITER :\n\
         *BUS_DELIMITER []\n*T_UNIT 1 NS\n*C_UNIT {cu}\n*R_UNIT {ru}\n*L_UNIT 1 HENRY\n\n"
    )
}

fn one_net(body: &str, cu: &str, ru: &str) -> Spef {
    Spef::parse(&format!("{}{}", hdr(cu, ru), body))
}

/// **Corner triplets.** `min:typ:max` is written whenever more than one corner is extracted.
/// Parsed naively the whole file yields nets with no capacitance and no resistance — which a
/// timer reads as ideal interconnect, on every net, with no warning.
#[test]
fn corner_triplets_are_read_not_dropped() {
    let s = one_net(
        "*NAME_MAP\n*1 neta\n*2 netb\n\n*D_NET *1 10:11:12\n*CONN\n*I *2:A I *L 1.5:1.6:1.7\n\
         *CAP\n1 *1 8:9:10\n*RES\n1 *1 *2:A 20:21:22\n*END\n",
        "1 FF",
        "1 OHM",
    );
    let n = s.nets.get("neta").expect("net parses");
    assert_eq!(n.cap_ff, 10.0, "*D_NET total");
    assert_eq!(n.res_ohm, 20.0, "*RES");
    assert_eq!(n.ground.len(), 1, "grounded *CAP");
    assert_eq!(n.ground[0].1, 8.0);
    assert_eq!(n.conns.first().map(|c| c.cap_ff), Some(1.5), "*L pin load");
    // and the read declares which corner it took, rather than pretending to be corner-aware
    assert!(s.triplets, "a triplet file must say so");
    assert!(s.health().unwrap().contains("triplet"));
}

/// **Keyword case.** The grammar is case-insensitive. Upper-case-only matching does not
/// mis-read such a file, it returns an empty design.
#[test]
fn keywords_are_case_insensitive() {
    let s = one_net(
        "*name_map\n*1 neta\n\n*d_net *1 10\n*cap\n1 *1 8\n*res\n1 *1 *1 5\n*end\n",
        "1 FF",
        "1 OHM",
    );
    let n = s.nets.get("neta").expect("a lower-case file is still a file");
    assert_eq!(n.cap_ff, 10.0);
    assert_eq!(n.ground.len(), 1);
}

/// **Name escaping.** A hierarchical or bussed name arrives escaped; left escaped it matches
/// no net in the design, so the net silently carries no parasitics.
#[test]
fn escaped_names_are_unescaped() {
    let s = one_net(
        "*NAME_MAP\n*1 u_a\\.q\\[0\\]\n\n*D_NET *1 10\n*CAP\n1 *1 8\n*END\n",
        "1 FF",
        "1 OHM",
    );
    assert!(s.nets.contains_key("u_a.q[0]"), "got {:?}", s.nets.keys().collect::<Vec<_>>());
}

/// And the writer escapes on the way out, so a name survives the round trip rather than
/// becoming a syntax error for the next reader.
#[test]
fn escaping_round_trips() {
    let src = one_net(
        "*NAME_MAP\n*1 u_a\\.q\\[0\\]\n\n*D_NET *1 10\n*CAP\n1 *1 8\n*END\n",
        "1 FF",
        "1 OHM",
    );
    let text = src.to_spef(&Default::default());
    assert!(text.contains("u_a\\.q\\[0\\]"), "writer must escape: {text}");
    let back = Spef::parse(&text);
    assert!(back.nets.contains_key("u_a.q[0]"));
    assert_eq!(back.nets["u_a.q[0]"].cap_ff, 10.0);
}

/// **Comments.** `//` is legal anywhere. Left in place its tokens change an entry's field
/// count, and field count is how this format tells a grounded cap from a coupling one — so a
/// trailing comment silently deletes the capacitance.
#[test]
fn line_comments_are_stripped() {
    let s = one_net(
        "// leading comment\n*NAME_MAP\n*1 neta\n\n*D_NET *1 10\n*CAP\n1 *1 8 // trailing\n*END\n",
        "1 FF",
        "1 OHM",
    );
    let n = s.nets.get("neta").expect("net parses");
    assert_eq!(n.ground.len(), 1, "a comment must not eat the ground cap");
    assert_eq!(n.ground[0].1, 8.0);
}

/// Units are applied, in both directions, including the scaled ones.
#[test]
fn units_scale() {
    let pf = one_net("*NAME_MAP\n*1 n\n\n*D_NET *1 0.01\n*CAP\n1 *1 0.008\n*END\n", "1 PF", "1 OHM");
    assert_eq!(pf.nets["n"].cap_ff, 10.0, "PF -> fF");
    let ko = one_net(
        "*NAME_MAP\n*1 n\n*2 m\n\n*D_NET *1 1\n*RES\n1 *1 *2 0.02\n*END\n",
        "1 FF",
        "1 KOHM",
    );
    assert_eq!(ko.nets["n"].res_ohm, 20.0, "KOHM -> ohm");
}

/// The name map is optional in IEEE 1481; literal names must work.
#[test]
fn a_file_without_a_name_map_parses() {
    let s = one_net("*D_NET neta 10\n*CAP\n1 neta 8\n*END\n", "1 FF", "1 OHM");
    assert_eq!(s.nets["neta"].cap_ff, 10.0);
}

/// Scientific notation is a number.
#[test]
fn scientific_notation_parses() {
    let s = one_net("*NAME_MAP\n*1 n\n\n*D_NET *1 1.0e-2\n*CAP\n1 *1 8.5e-3\n*END\n", "1 FF", "1 OHM");
    assert!((s.nets["n"].cap_ff - 0.01).abs() < 1e-12);
    assert!((s.nets["n"].ground[0].1 - 0.0085).abs() < 1e-12);
}

/// **The tell.** A parser that cannot fail must at least count what it did not understand,
/// or the next gap is as quiet as the last four.
#[test]
fn unreadable_entries_are_counted_not_ignored() {
    let clean = one_net("*NAME_MAP\n*1 n\n\n*D_NET *1 10\n*CAP\n1 *1 8\n*END\n", "1 FF", "1 OHM");
    assert_eq!(clean.skipped, 0);
    assert!(clean.health().is_none(), "a clean read reports nothing");

    let junk = one_net(
        "*NAME_MAP\n*1 n\n\n*D_NET *1 10\n*CAP\n1 *1 8\n2 *1 not_a_number\n*END\n",
        "1 FF",
        "1 OHM",
    );
    assert_eq!(junk.skipped, 1, "the unreadable entry is counted");
    assert!(junk.health().unwrap().contains("not understood"));

    // The worst case of all: nothing parsed at all, which must never look like success.
    let empty = Spef::parse("*SPEF \"IEEE 1481-1999\"\n");
    assert!(empty.health().unwrap().contains("no nets"));
}

/// **Reduced parasitics.** `*R_NET` carries a pi-model instead of an RC network. We do not
/// model it — but a file of them must not read as "an empty design" with no clue why, which is
/// indistinguishable from a design with no parasitics at all.
#[test]
fn reduced_nets_are_reported_not_silently_empty() {
    let s = one_net(
        "*NAME_MAP\n*1 neta\n\n*R_NET *1 10\n*DRIVER *2:Y\n*RC 3 5\n*END\n",
        "1 FF",
        "1 OHM",
    );
    assert!(s.nets.is_empty(), "no detailed net to read");
    assert_eq!(s.reduced, 1);
    let h = s.health().expect("must not look like a clean empty read");
    assert!(h.contains("*R_NET"), "{h}");
    assert!(h.contains("pi-model"), "{h}");
}

/// Power-net parasitics are skipped by design and must not be mistaken for signal nets, nor
/// leave the reader mid-block so the next `*D_NET` inherits their entries.
#[test]
fn power_net_blocks_are_skipped_cleanly() {
    let s = one_net(
        "*NAME_MAP\n*1 VDD\n*2 neta\n\n*D_PNET *1 99\n*CAP\n1 *1 77\n*END\n\
         \n*D_NET *2 10\n*CAP\n1 *2 8\n*END\n",
        "1 FF",
        "1 OHM",
    );
    assert_eq!(s.pnets, 1);
    assert_eq!(s.nets.len(), 1, "only the signal net");
    let n = s.nets.get("neta").expect("the signal net after a power block");
    assert_eq!(n.cap_ff, 10.0);
    assert_eq!(n.ground.len(), 1, "must not inherit the power block's entries");
    assert_eq!(n.ground[0].1, 8.0);
}
