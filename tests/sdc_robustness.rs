//! Robustness for the SDC reader, driven by the commands our own constraint files actually use.
//!
//! `tests/data/sdc_commands.txt` holds 56 commands mined from 154 real `.sdc` files across this
//! organisation's IP repos, SoC projects and OpenLane/LibreLane flows. SDC is Tcl and its
//! reference implementations are GPLv3, so the inventory comes from the constraint files rather
//! than from anyone's parser — which is the list that matters, being what we are handed.
//!
//! SDC differs from LEF/DEF/Liberty in one important way: **not modelling a command is a
//! legitimate position**, and this reader already records every one it passes over. So the
//! property is not "nothing changes" alone — it is also "what was passed over is *reported*, and
//! reported in a way that distinguishes a constraint which moves a slack from one that does
//! not". `set_dont_touch` costs a timer nothing; `set_driving_cell` sets the slew every input
//! path starts from, and our own files use it 480 times against 65 uses of
//! `set_input_transition`.

use vyges_loom::sdc::Sdc;

/// The base file must always parse; a case that does not is a test bug, not a finding.
fn parse(text: &str) -> Sdc {
    Sdc::parse(text).expect("valid SDC")
}

/// The mined command list, with the trailing `# count` comment stripped.
fn commands() -> Vec<&'static str> {
    include_str!("data/sdc_commands.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_whitespace().next())
        .collect()
}

/// Commands this reader models — injecting one of these legitimately changes the result.
const HANDLED: &[&str] = &[
    "create_clock",
    "create_generated_clock",
    "set_input_delay",
    "set_output_delay",
    "set_clock_uncertainty",
    "set_clock_latency",
    "set_input_transition",
    "set_load",
    "set_timing_derate",
    "set_false_path",
    "set_multicycle_path",
    "set_clock_groups",
    "set_clock_group",
    "set_units",
    "set_unit",
];

/// A constraint file exercising everything this reader claims to read.
fn good_sdc(extra: &str) -> String {
    format!(
        "# a representative constraint file\n\
         create_clock -name clk -period 10.0 [get_ports clk]\n\
         set_clock_uncertainty -setup 0.2 [get_clocks clk]\n\
         set_clock_uncertainty -hold 0.05 [get_clocks clk]\n\
         set_clock_latency 0.5 [get_clocks clk]\n\
         set_input_delay 2.0 -clock clk [all_inputs]\n\
         set_output_delay 3.0 -clock clk [all_outputs]\n\
         set_input_transition 0.15 [all_inputs]\n\
         set_load 0.05 [all_outputs]\n\
         set_timing_derate -late 1.05\n\
         set_false_path -from [get_ports rst_n]\n\
         {extra}"
    )
}

/// Everything this reader is supposed to extract, as one comparable value.
fn fingerprint(s: &Sdc) -> String {
    format!(
        "clocks={:?} in={:?} out={:?} unc={:.6}/{:.6} lat={:.6} tr={:?} load={:?} \
         derate={:?}/{:?} exc={} groups={:?}",
        s.clocks
            .iter()
            .map(|c| format!("{}:{}:{:.6}", c.name, c.source, c.period))
            .collect::<Vec<_>>(),
        s.input_delays
            .iter()
            .map(|d| format!("{:.6}:{}:{:?}", d.value, d.default, d.ports))
            .collect::<Vec<_>>(),
        s.output_delays
            .iter()
            .map(|d| format!("{:.6}:{}:{:?}", d.value, d.default, d.ports))
            .collect::<Vec<_>>(),
        s.setup_uncertainty,
        s.hold_uncertainty,
        s.clock_latency,
        s.input_transition,
        s.load,
        s.late_derate,
        s.early_derate,
        s.exceptions.len(),
        s.async_groups,
    )
}

#[test]
fn the_known_good_file_reads_as_expected() {
    let s = parse(&good_sdc(""));
    assert_eq!(s.clocks.len(), 1);
    assert_eq!(s.clocks[0].name, "clk");
    assert_eq!(s.clocks[0].period, 10.0);
    assert_eq!(s.setup_uncertainty, 0.2);
    assert_eq!(s.hold_uncertainty, 0.05);
    assert_eq!(s.clock_latency, 0.5);
    assert_eq!(s.input_delays.len(), 1);
    assert_eq!(s.input_delays[0].value, 2.0);
    assert!(s.input_delays[0].default, "[all_inputs] is the default budget");
    assert_eq!(s.output_delays[0].value, 3.0);
    assert_eq!(s.input_transition, Some(0.15));
    assert_eq!(s.load, Some(0.05));
    assert_eq!(s.late_derate, Some(1.05));
    assert_eq!(s.exceptions.len(), 1);
    assert!(s.health().is_none(), "nothing ignored that matters: {:?}", s.health());
}

/// **The generated sweep.** Every command our constraint files use, injected into a known-good
/// file. An unmodelled one must change nothing we parsed.
#[test]
fn no_unmodelled_command_can_perturb_what_we_read() {
    let base = fingerprint(&parse(&good_sdc("")));
    let mut broke = Vec::new();
    for c in commands() {
        if HANDLED.contains(&c) {
            continue;
        }
        for body in [
            format!("{c}\n"),
            format!("{c} 1.0\n"),
            format!("{c} -foo 1.0 [get_ports {{a b}}]\n"),
            format!("{c} [all_inputs]\n"),
        ] {
            if fingerprint(&parse(&good_sdc(&body))) != base {
                broke.push(format!("{c}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} command(s) perturbed the constraints:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// **And every unmodelled command must be RECORDED.** Silence is the failure mode: a constraint
/// that vanished is indistinguishable from one that was never written.
#[test]
fn every_unmodelled_command_is_recorded() {
    let mut missing = Vec::new();
    for c in commands() {
        if HANDLED.contains(&c) {
            continue;
        }
        let s = parse(&good_sdc(&format!("{c} 1.0 [get_ports a]\n")));
        if !s.ignored.iter().any(|i| i == c) {
            missing.push(c);
        }
    }
    assert!(
        missing.is_empty(),
        "{} command(s) vanished without being recorded: {}",
        missing.len(),
        missing.join(", ")
    );
}

/// **The one that matters most.** `set_driving_cell` is the most-used input constraint in our
/// own files — 480 uses against 65 of `set_input_transition` — and we do not model it. That is
/// a defensible position; reporting it in the same undifferentiated list as `set_dont_touch` is
/// not, because only one of the two can move a slack.
#[test]
fn timing_affecting_constraints_are_distinguished_from_benign_ones() {
    let benign = parse(&good_sdc(
        "set_dont_touch [get_cells u1]\nset_max_area 0\nset_size_only [get_cells u2]\n",
    ));
    assert!(!benign.ignored.is_empty(), "they are still recorded");
    assert!(
        benign.ignored_affecting_timing().is_empty(),
        "synthesis directives cannot move a slack: {:?}",
        benign.ignored_affecting_timing()
    );
    assert!(benign.health().is_none());

    let matters = parse(&good_sdc(
        "set_driving_cell -lib_cell INV_X1 [all_inputs]\n\
         set_case_analysis 0 [get_ports test_en]\n\
         set_dont_touch [get_cells u1]\n",
    ));
    let a = matters.ignored_affecting_timing();
    assert!(a.contains(&"set_driving_cell"), "{a:?}");
    assert!(a.contains(&"set_case_analysis"), "{a:?}");
    assert!(!a.contains(&"set_dont_touch"), "{a:?}");
    let h = matters.health().expect("must report");
    assert!(h.contains("set_driving_cell"), "{h}");

    // Forty ports constrained the same way is one finding, not forty.
    let many = parse(&good_sdc(
        &(0..40)
            .map(|i| format!("set_driving_cell -lib_cell INV_X1 [get_ports p{i}]\n"))
            .collect::<String>(),
    ));
    assert_eq!(many.ignored_affecting_timing(), vec!["set_driving_cell"]);
}

/// Tcl mechanics the reader promises: `\` continuations, `#` comments, and `set`/`$var`.
/// A continuation misread splits one command into two, and the second half is nonsense.
#[test]
fn tcl_mechanics_hold() {
    let s = parse(
        "set period 8.0\n\
         # a comment mentioning create_clock -period 999\n\
         create_clock -name clk \\\n  -period $period \\\n  [get_ports clk]\n",
    );
    assert_eq!(s.clocks.len(), 1, "one clock, not two: {:?}", s.clocks);
    assert_eq!(s.clocks[0].period, 8.0, "the variable resolved");
    assert_eq!(s.clocks[0].name, "clk");
}

/// An empty or comment-only file is not an error, and must not invent constraints.
#[test]
fn an_empty_file_yields_nothing() {
    for text in ["", "# nothing here\n", "\n\n   \n"] {
        let s = parse(text);
        assert!(s.clocks.is_empty() && s.input_delays.is_empty() && s.exceptions.is_empty());
        assert!(s.health().is_none());
    }
}

/// **A flow parameterises its constraints.** OpenLane writes
/// `-period $::env(CLOCK_PERIOD)`, and 8 of this organisation's 154 constraint files do exactly
/// that. Resolving only `$var` left the period empty and failed the WHOLE file — every other
/// constraint in it lost along with the clock.
#[test]
fn env_variables_resolve_like_tcl() {
    // SAFETY: single-threaded within this test, and the name is specific to it.
    unsafe { std::env::set_var("VYGES_TEST_CLOCK_PERIOD", "12.5") };
    for form in ["$::env(VYGES_TEST_CLOCK_PERIOD)", "$env(VYGES_TEST_CLOCK_PERIOD)"] {
        let s = parse(&format!("create_clock -name clk -period {form} [get_ports clk]\n"));
        assert_eq!(s.clocks.len(), 1, "{form}");
        assert_eq!(s.clocks[0].period, 12.5, "{form}");
    }
    // a `set` of the same name wins over the environment, as in Tcl
    let s = parse(
        "set VYGES_TEST_CLOCK_PERIOD 4.0\n\
         create_clock -name clk -period $::env(VYGES_TEST_CLOCK_PERIOD) [get_ports clk]\n",
    );
    assert_eq!(s.clocks[0].period, 4.0);
}

/// **An unresolved variable must not vanish.** Substituting nothing deletes the token and the
/// next argument slides into its place — so `-period $undefined 5` would silently yield a clock
/// with a period of 5, which is a plausible number and completely wrong. Keeping the text makes
/// it fail to parse, and the error names the variable.
#[test]
fn an_unresolved_variable_does_not_let_the_next_argument_become_the_value() {
    let e = Sdc::parse("create_clock -name clk -period $undefined 5\n")
        .expect_err("must not silently invent a period");
    let msg = e.to_string();
    assert!(msg.contains("$undefined"), "the error must name the variable: {msg}");

    // the same shape a real file had: the port expression must never become the period
    let e = Sdc::parse("create_clock -name clk -period $clk_period [get_ports clk]\n")
        .expect_err("must not read a port list as a period");
    assert!(e.to_string().contains("$clk_period"), "{e}");
}

/// **Quartus and several board vendors write a unit suffix**: `-period 20.000ns`. A bare
/// `parse::<f64>()` rejects it and takes the whole file down with it — 6 of our 154 files.
#[test]
fn a_period_may_carry_a_unit_suffix() {
    let cases = [("20.000ns", 20.0), ("20ns", 20.0), ("500ps", 0.5), ("1us", 1000.0)];
    for (text, want) in cases {
        let s = parse(&format!("create_clock -name c -period {text} [get_ports clk]\n"));
        assert!(
            (s.clocks[0].period - want).abs() < 1e-9,
            "{text}: want {want} ns, got {}",
            s.clocks[0].period
        );
    }
    // a bare number keeps its meaning — the file's own time unit
    assert_eq!(parse("create_clock -name c -period 20 [get_ports clk]\n").clocks[0].period, 20.0);
}

/// The design-rule constraints are timing findings, not synthesis directives: a `set_max_*`
/// that is not modelled means those violations are never reported, which reads as a clean run.
#[test]
fn design_rule_limits_count_as_timing_affecting() {
    let s = parse(&good_sdc(
        "set_max_transition 0.75 [current_design]\n\
         set_max_capacitance 0.2 [all_outputs]\n\
         set_max_fanout 10 [current_design]\n",
    ));
    let a = s.ignored_affecting_timing();
    for c in ["set_max_transition", "set_max_capacitance", "set_max_fanout"] {
        assert!(a.contains(&c), "{c} missing from {a:?}");
    }
}

/// **Every command in the spec is classified, one way or the other.** An unmodelled constraint
/// that is neither known to move a slack nor known to be harmless is an *unreviewed* one, and
/// this is what stops the list going stale as SDC grows.
#[test]
fn every_spec_command_is_either_modelled_or_classified() {
    // Object-access and Tcl helpers are not constraints; they appear inside other commands.
    const NOT_A_CONSTRAINT: &[&str] = &[
        "all_clocks", "all_inputs", "all_outputs", "all_registers", "current_design",
        "current_instance", "get_cells", "get_clocks", "get_lib_cells", "get_lib_pins",
        "get_libs", "get_nets", "get_pins", "get_ports", "expr", "list", "concat",
        "set_hierarchy_separator", "read_liberty", "read_verilog", "link_design",
        "report_checks", "report_tns", "write_sdc", "sta", "set_propagated_clocks",
        "append_to_collection", "group_name",
        // mining noise: a signal name that began a continuation line
        "axi_araddr_i",
    ];
    let mut unreviewed = Vec::new();
    for c in commands() {
        if HANDLED.contains(&c) || NOT_A_CONSTRAINT.contains(&c) {
            continue;
        }
        // A constraint is reviewed when injecting it produces a classification either way.
        let s = parse(&good_sdc(&format!("{c} 1.0 [get_ports a]\n")));
        let affects = !s.ignored_affecting_timing().is_empty();
        let benign = vyges_loom::sdc::BENIGN_FOR_TIMING.contains(&c);
        if !affects && !benign {
            unreviewed.push(c);
        }
    }
    assert!(
        unreviewed.is_empty(),
        "{} command(s) are unmodelled and unclassified — decide whether each can move a slack:\n  {}",
        unreviewed.len(),
        unreviewed.join("\n  ")
    );
}

/// **SDC accepts the singular and plural spellings interchangeably**, and tools emit both.
/// Matching only one silently ignores the other — and for `set_clock_group` that loses an
/// asynchronous-clock declaration, so paths between clocks that never relate get checked and
/// report violations that do not exist. Found by diffing our inventory against SDC 2.1.
#[test]
fn singular_and_plural_command_spellings_are_equivalent() {
    for (plural, singular) in [("set_clock_groups", "set_clock_group")] {
        let p = parse(&good_sdc(&format!(
            "{plural} -asynchronous -group {{clk}} -group {{clk2}}\n"
        )));
        let s = parse(&good_sdc(&format!(
            "{singular} -asynchronous -group {{clk}} -group {{clk2}}\n"
        )));
        assert_eq!(p.async_groups, s.async_groups, "{plural} vs {singular}");
        assert_eq!(p.async_groups.len(), 2, "both groups read: {:?}", p.async_groups);
        assert!(
            !s.ignored.iter().any(|i| i == singular),
            "{singular} must be handled, not ignored"
        );
    }
    // and the same for units
    let a = parse("set_units -time 1ns\ncreate_clock -name c -period 4 [get_ports clk]\n");
    let b = parse("set_unit -time 1ns\ncreate_clock -name c -period 4 [get_ports clk]\n");
    assert_eq!(a.clocks[0].period, b.clocks[0].period);
}
