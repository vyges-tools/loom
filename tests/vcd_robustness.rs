//! Robustness for the activity readers (VCD today; SAIF and FST share the invariants).
//!
//! These produce **switching activity**, which feeds dynamic power. A wrong toggle count is not
//! a crash and not a zero — it is a plausible power number that is simply not the design's, and
//! nothing downstream can tell. Every case below was silent, and each inflated or deflated
//! activity rather than failing.

use vyges_loom::vcd::Vcd;

fn vcd(decls: &str, body: &str) -> String {
    format!(
        "$timescale 1ns $end\n$scope module top $end\n{decls}$upscope $end\n\
         $enddefinitions $end\n{body}"
    )
}

fn toggles(v: &Vcd, net: &str) -> u64 {
    v.idx.resolve(net).unwrap_or(0)
}

#[test]
fn a_clean_dump_counts_transitions_not_values() {
    let v = Vcd::parse(&vcd(
        "$var wire 1 ! clk $end\n",
        "#0\n0!\n#10\n1!\n#20\n0!\n#30\n1!\n#40\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "clk"), 3, "the initial value is not a transition");
    assert!((v.sim_time_s - 40e-9).abs() < 1e-18);
}

/// **`x` and `z` are not levels.** They are the absence of one, so they neither count as a
/// transition nor replace what was known. Counting them inflated activity — and therefore
/// dynamic power — on every net that is ever undriven or uninitialised.
#[test]
fn unknown_values_do_not_count_as_transitions() {
    // the net really did change, once, through an unknown
    let through = Vcd::parse(&vcd("$var wire 1 ! n $end\n", "#0\n0!\n#10\nx!\n#20\nz!\n#30\n1!\n")).unwrap();
    assert_eq!(toggles(&through, "n"), 1, "0 -> x -> z -> 1 is ONE toggle");

    // and here it did not change at all
    let back = Vcd::parse(&vcd("$var wire 1 ! n $end\n", "#0\n0!\n#10\nx!\n#20\n0!\n")).unwrap();
    assert_eq!(toggles(&back, "n"), 0, "0 -> x -> 0 is no toggle");
}

/// **`$dumpoff` writes `x` over every signal.** That is a statement about the dumper, not the
/// circuit, and counting it charged every net in the design for transitions it never made.
#[test]
fn a_dumpoff_window_contributes_no_activity() {
    let v = Vcd::parse(&vcd(
        "$var wire 1 ! clk $end\n",
        "#0\n$dumpvars\n0!\n$end\n#10\n$dumpoff\nx!\n$end\n#20\n$dumpon\n0!\n$end\n#30\n1!\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "clk"), 1, "only the real 0->1 at #30");
}

/// **One identifier code may name several signals.** A net and its port alias share a code, and
/// dumpers emit that routinely. Keyed singly, the first declaration was overwritten and that
/// signal reported a toggle rate of zero while its alias reported the truth.
#[test]
fn an_aliased_identifier_credits_every_name() {
    let v = Vcd::parse(&vcd(
        "$var wire 1 ! clk $end\n$var wire 1 ! clk_alias $end\n",
        "#0\n0!\n#10\n1!\n#20\n0!\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "clk"), 2);
    assert_eq!(toggles(&v, "clk_alias"), 2, "both names are the same signal");
}

/// **A `$comment` body contains anything at all**, including things shaped exactly like a
/// timestamp and a value change. Read as data, commented-out stimulus became real activity.
#[test]
fn a_comment_body_is_not_activity() {
    let v = Vcd::parse(&vcd(
        "$var wire 1 ! clk $end\n",
        "$comment\n#999\n1!\n0!\n1!\n$end\n#0\n0!\n#10\n1!\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "clk"), 1);
    assert!((v.sim_time_s - 10e-9).abs() < 1e-18, "and #999 is not the end of the sim");
}

/// A real-valued signal is not bit-decomposable, so it counts one toggle per CHANGE — and the
/// initial dump is not a change.
#[test]
fn a_real_signal_counts_changes_not_dumps() {
    let v = Vcd::parse(&vcd("$var real 64 ! v $end\n", "#0\nr1.5 !\n#10\nr2.5 !\n#20\nr2.5 !\n")).unwrap();
    assert_eq!(toggles(&v, "v"), 1, "one change; the dump and the repeat are not");
}

/// **VCD drops leading zeros on vector values**: `b1` on a 4-bit net is `0001`. Comparing the
/// strings as written would report the wrong bits flipping.
#[test]
fn vector_values_are_left_padded_before_comparison() {
    let v = Vcd::parse(&vcd(
        "$var wire 4 \" d [3:0] $end\n",
        "#0\nb0000 \"\n#10\nb1 \"\n#20\nb1111 \"\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "d[0]"), 1, "0 -> 1 -> 1");
    assert_eq!(toggles(&v, "d[3]"), 1, "0 -> 0 -> 1");
    assert_eq!(toggles(&v, "d[1]"), 1);
}

/// Per bit, an unknown behaves exactly as it does on a scalar.
#[test]
fn unknown_bits_in_a_vector_do_not_count() {
    let v = Vcd::parse(&vcd(
        "$var wire 2 \" d [1:0] $end\n",
        "#0\nb00 \"\n#10\nbx0 \"\n#20\nb10 \"\n",
    ))
    .unwrap();
    assert_eq!(toggles(&v, "d[1]"), 1, "0 -> x -> 1 is one toggle");
    assert_eq!(toggles(&v, "d[0]"), 0);
}

/// **The windowing invariant.** Activity over the whole dump must equal the activity of its
/// parts — an internal property that needs no reference, and that exercises the window
/// arithmetic the power engine relies on when it analyses a slice of a long run.
#[test]
fn windows_partition_the_activity() {
    let src = vcd(
        "$var wire 1 ! a $end\n$var wire 2 \" d [1:0] $end\n",
        "#0\n0!\nb00 \"\n#10\n1!\n#20\nb11 \"\n#30\n0!\n#40\nb01 \"\n#50\n",
    );
    let whole = Vcd::parse(&src).unwrap();
    let first = Vcd::parse_windowed(&src, Some((0.0, Some(25e-9)))).unwrap();
    let second = Vcd::parse_windowed(&src, Some((25e-9, None))).unwrap();
    for net in ["a", "d[0]", "d[1]"] {
        assert_eq!(
            toggles(&whole, net),
            toggles(&first, net) + toggles(&second, net),
            "{net}: whole {} != {} + {}",
            toggles(&whole, net),
            toggles(&first, net),
            toggles(&second, net)
        );
    }
}

// ── SAIF ────────────────────────────────────────────────────────────────────────────────────

use vyges_loom::saif::Saif;

fn saif(body: &str) -> String {
    format!(
        "(SAIFILE\n (SAIFVERSION \"2.0\")\n (DIRECTION \"backward\")\n (TIMESCALE 1 ns)\n \
         (DURATION 1000)\n{body})\n"
    )
}

/// **SAIF escapes punctuation in names**: a bussed net is written `i\[0\]`. Left escaped it
/// matches nothing in the netlist, so the net resolves to no activity and its power is computed
/// from a default estimate instead of the measurement — quietly, because "no activity recorded"
/// and "a net that never toggles" look identical from the outside.
#[test]
fn saif_names_are_unescaped_to_match_the_netlist() {
    let s = Saif::parse(&saif(
        " (INSTANCE top\n  (NET\n   (i\\[0\\] (T0 500) (T1 500) (TC 40))\n   \
         (u\\.a (T0 500) (T1 500) (TC 4))\n   (plain (T0 500) (T1 500) (TC 2))\n  )\n )\n",
    ))
    .expect("valid SAIF");
    assert_eq!(s.idx.toggles.get("top.i[0]"), Some(&40), "{:?}", s.idx.toggles.keys().collect::<Vec<_>>());
    assert_eq!(s.idx.toggles.get("top.u.a"), Some(&4));
    assert_eq!(s.idx.toggles.get("top.plain"), Some(&2), "an unescaped name is untouched");
}

/// Nested `INSTANCE` groups build the hierarchical path, and every level's nets are kept.
#[test]
fn saif_hierarchy_builds_full_paths() {
    let s = Saif::parse(&saif(
        " (INSTANCE top\n  (NET (a (T0 500) (T1 500) (TC 2)))\n  \
         (INSTANCE dut\n   (NET (b (T0 500) (T1 500) (TC 6)))\n   \
         (INSTANCE ff0\n    (NET (CK (T0 500) (T1 500) (TC 10)))\n   )\n  )\n )\n",
    ))
    .expect("valid SAIF");
    assert_eq!(s.idx.toggles.get("top.a"), Some(&2));
    assert_eq!(s.idx.toggles.get("top.dut.b"), Some(&6));
    assert_eq!(s.idx.toggles.get("top.dut.ff0.CK"), Some(&10));
}

/// `TC` is a count and `DURATION` a span; the rate is the one divided by the other, in the
/// file's own timescale. Getting the timescale wrong scales every power number in the design.
#[test]
fn saif_toggle_rate_uses_the_declared_timescale() {
    // 1000 ns of dump, 40 toggles -> 40 / 1e-6 s
    let ns = Saif::parse(&saif(" (INSTANCE top (NET (a (T0 500) (T1 500) (TC 40))))\n"))
        .expect("valid SAIF");
    assert!((ns.sim_time_s - 1.0e-6).abs() < 1e-15, "got {:e}", ns.sim_time_s);
    assert!((ns.toggle_rate("top.a") - 40.0e6).abs() < 1.0, "got {:e}", ns.toggle_rate("top.a"));

    // the same file in picoseconds is a thousand times shorter, and the rate a thousand times up
    let ps = Saif::parse(
        &saif(" (INSTANCE top (NET (a (T0 500) (T1 500) (TC 40))))\n").replace("1 ns", "1 ps"),
    )
    .expect("valid SAIF");
    assert!((ps.sim_time_s - 1.0e-9).abs() < 1e-18, "got {:e}", ps.sim_time_s);
}

// ── cross-format ────────────────────────────────────────────────────────────────────────────

/// **The strongest check available here: two independent readers, one answer.**
///
/// `tests/fixtures/counter.vcd` and `counter.fst` are the same simulation in a text and a binary
/// format, read by two separately written parsers — one a line reader, one a clean-room decoder
/// of a compressed binary layout. If both are right they agree exactly; if either drifts, they
/// cannot. No reference waveform is needed and no expected value is stored, which is why this
/// survives changes to either reader that a golden number would not.
#[cfg(feature = "fst")]
#[test]
fn the_text_and_binary_readers_agree_on_the_same_simulation() {
    use vyges_loom::fst::Fst;
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/counter");
    let v = Vcd::load(&format!("{dir}.vcd")).expect("vcd fixture");
    let f = Fst::load(&format!("{dir}.fst")).expect("fst fixture");

    assert!(!v.idx.toggles.is_empty(), "the fixture must actually contain activity");
    assert!(
        (v.sim_time_s - f.sim_time_s).abs() < 1e-15,
        "sim time: vcd {:e} vs fst {:e}",
        v.sim_time_s,
        f.sim_time_s
    );
    assert_eq!(
        v.idx.toggles.len(),
        f.idx.toggles.len(),
        "different net counts: {} vs {}",
        v.idx.toggles.len(),
        f.idx.toggles.len()
    );
    let mut disagree = Vec::new();
    for (path, n) in &v.idx.toggles {
        match f.idx.toggles.get(path) {
            Some(m) if m == n => {}
            Some(m) => disagree.push(format!("{path}: vcd={n} fst={m}")),
            None => disagree.push(format!("{path}: vcd={n} fst=absent")),
        }
    }
    assert!(disagree.is_empty(), "{}", disagree.join("\n"));
}

/// IEEE 1164 nine-state levels, counted the same by both readers.
///
/// `H` and `L` are the weak 1 and weak 0 and a transition to one is a real transition. `U`, `W`,
/// `X`, `Z` and `-` are the absence of one: they count nothing and leave the last known value in
/// place. A VHDL dump (nvc, GHDL) is written entirely in these, and the two readers used to
/// disagree on every one — the VCD reader dropped `H`/`L` scalar changes on the floor, the FST
/// reader counted `U` and `W` as if they were levels.
///
/// The counts below are derived by hand from the fixture, not from either reader.
#[test]
fn nine_state_levels_count_as_weak_ones_and_zeros() {
    let v = vyges_loom::vcd::Vcd::load(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/ninestate.vcd"
    ))
    .expect("ninestate.vcd");
    let got = |n: &str| v.idx.toggles.get(n).copied().unwrap_or(0);

    assert_eq!(got("top.clk"), 4, "0,1,0,1,0");
    // U -> H -> L -> W -> H: the U seeds nothing, the W is skipped without becoming the
    // baseline, so H->L and the L->H across it are the only two transitions.
    assert_eq!(got("top.weak"), 2);
    // UUUU -> HLZ0 -> LH1X -> 0011 -> ZZZZ, per bit, MSB first
    assert_eq!(got("top.bus[3]"), 1);
    assert_eq!(got("top.bus[2]"), 2);
    assert_eq!(got("top.bus[1]"), 0);
    assert_eq!(got("top.bus[0]"), 1);
    // a real is one net, not 64: 1.0 -> 2.5 -> 2.5 -> 3.5 -> 3.5
    assert_eq!(got("top.freq"), 2);
    assert!(!v.idx.toggles.keys().any(|k| k.starts_with("top.freq[")), "reals are not bit-expanded");
    // a string carries no switching activity
    assert_eq!(got("top.label"), 0);
}

/// **A vector dumped one bit at a time keeps its indices.** ModelSim, Quartus and others do not
/// emit a vector as one wide `$var`; they emit one 1-bit `$var` per bit, each with its own
/// identifier and a bit-select. Dropping the select merges every bit onto one net — and because
/// each reader keeps one last-known value per NAME, the bits then overwrite each other's history
/// and the total is neither the vector's activity nor any bit's.
///
/// A one-bit RANGE (`[0:0]`) is a different declaration: the signal's whole width is one bit and
/// it is called `w`, not `w[0]`. The colon is what tells them apart.
#[test]
fn a_vector_dumped_one_bit_per_var_keeps_its_indices() {
    let v = vyges_loom::vcd::Vcd::parse(
        "$timescale 1ns $end\n\
         $scope module tb $end\n\
         $var wire 1 ) r [2] $end\n\
         $var wire 1 * r [1] $end\n\
         $var wire 1 + r [0] $end\n\
         $var wire 1 , w [0:0] $end\n\
         $upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\n0)\n0*\n0+\n0,\n$end\n\
         #10\n1)\n1+\n1,\n\
         #20\n0)\n1*\n",
    )
    .expect("vcd");
    let got = |n: &str| v.idx.toggles.get(n).copied().unwrap_or(0);
    assert_eq!(got("tb.r[2]"), 2, "0 -> 1 -> 0");
    assert_eq!(got("tb.r[1]"), 1);
    assert_eq!(got("tb.r[0]"), 1);
    assert!(!v.idx.toggles.contains_key("tb.r"), "the bits must not merge onto one net");
    // [0:0] is a range, not a bit-select: the signal is `w`
    assert_eq!(got("tb.w"), 1);
    assert!(!v.idx.toggles.contains_key("tb.w[0]"));
}

/// **One identifier naming a vector in several scopes credits every name.** A port carried down
/// a hierarchy shares one code, and dumpers emit that constantly. Keeping one last-value slot
/// per identifier let the first name consume the change and update it, so every other name
/// compared against the value just written and reported no activity at all.
#[test]
fn an_aliased_vector_identifier_credits_every_name() {
    let v = vyges_loom::vcd::Vcd::parse(
        "$timescale 1ns $end\n\
         $scope module a $end\n$var wire 4 ! bus [3:0] $end\n$upscope $end\n\
         $scope module b $end\n$var wire 4 ! bus [3:0] $end\n$upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\nb0000 !\n$end\n\
         #10\nb0011 !\n",
    )
    .expect("vcd");
    let got = |n: &str| v.idx.toggles.get(n).copied().unwrap_or(0);
    for scope in ["a", "b"] {
        assert_eq!(got(&format!("{scope}.bus[0]")), 1, "{scope}");
        assert_eq!(got(&format!("{scope}.bus[1]")), 1, "{scope}");
        assert_eq!(got(&format!("{scope}.bus[3]")), 0, "{scope}");
    }
}

/// **An unnamed scope contributes no path component.** Verilator writes one as the root of a
/// dump (`$scope module  $end`), and tokenising on whitespace makes the terminator look like the
/// name — so every path in the file came out under a scope called `$end` and matched nothing.
///
/// The scope is still PUSHED, empty: `$upscope` pops unconditionally, so skipping the push would
/// pop the parent instead and reparent everything after it. `other.rst` is what proves it.
#[test]
fn an_unnamed_scope_contributes_no_path_component() {
    let v = vyges_loom::vcd::Vcd::parse(
        "$timescale 1ns $end\n\
         $scope module  $end\n\
         $scope module top $end\n$var wire 1 ! clk $end\n$upscope $end\n\
         $upscope $end\n\
         $scope module other $end\n$var wire 1 \" rst $end\n$upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\n0!\n0\"\n$end\n\
         #10\n1!\n1\"\n",
    )
    .expect("vcd");
    let got = |n: &str| v.idx.toggles.get(n).copied().unwrap_or(0);
    assert_eq!(got("top.clk"), 1);
    assert_eq!(got("other.rst"), 1, "the empty scope must not swallow its sibling");
    assert!(
        !v.idx.toggles.keys().any(|k| k.starts_with("$end") || k.starts_with('.')),
        "no stray root component: {:?}",
        v.idx.toggles.keys().collect::<Vec<_>>()
    );
}

/// **A one-bit signal is often dumped in vector form.** `b0 <sym>` for a plain `reg` is the
/// writer's choice, not the signal's, and the same signal may appear in scalar form elsewhere in
/// the same file. Handling only the vector case dropped every such signal — declared, never
/// counted, and indistinguishable from a net that simply never moved.
#[test]
fn a_one_bit_signal_dumped_as_a_vector_still_counts() {
    let v = vyges_loom::vcd::Vcd::parse(
        "$timescale 1ns $end\n\
         $scope module tb $end\n$var reg 1 ! en $end\n$upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\nb0 !\n$end\n\
         #10\nb1 !\n\
         #20\n0!\n\
         #30\nb1 !\n",
    )
    .expect("vcd");
    // the two spellings share one baseline: 0 -> 1 -> 0 -> 1
    assert_eq!(v.idx.toggles.get("tb.en").copied().unwrap_or(0), 3);
}

/// **A fractional timestamp still advances the clock.** IEEE 1364 says `#` carries a whole
/// number of timescale units, but migen writes `#3.2` and viewers accept it. Parsed as an
/// integer it simply fails — and a failed parse used to leave the clock where it was, so a dump
/// whose every timestamp is fractional measured as zero-length and every toggle RATE taken from
/// it came out zero. Nothing about a rate of zero says the times were unreadable.
#[test]
fn a_fractional_timestamp_still_advances_the_clock() {
    let v = vyges_loom::vcd::Vcd::parse(
        "$timescale 1ns $end\n\
         $scope module tb $end\n$var wire 1 ! clk $end\n$upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\n0!\n$end\n\
         #3.2\n1!\n\
         #6.0\n0!\n\
         #15.0\n1!\n",
    )
    .expect("vcd");
    assert!((v.sim_time_s - 15.0e-9).abs() < 1e-18, "dump length: {:e}", v.sim_time_s);
    assert_eq!(v.idx.toggles.get("tb.clk").copied().unwrap_or(0), 3);
    // and the window boundaries land where the fractional times say they do
    let w = vyges_loom::vcd::Vcd::parse_windowed(
        "$timescale 1ns $end\n\
         $scope module tb $end\n$var wire 1 ! clk $end\n$upscope $end\n\
         $enddefinitions $end\n\
         #0\n$dumpvars\n0!\n$end\n\
         #3.2\n1!\n\
         #6.0\n0!\n\
         #15.0\n1!\n",
        Some((5.0e-9, None)),
    )
    .expect("vcd");
    assert_eq!(
        w.idx.toggles.get("tb.clk").copied().unwrap_or(0),
        2,
        "the change at 3.2 ns is before the window, the two after it are inside"
    );
}
