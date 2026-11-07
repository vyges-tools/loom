//! A real constraint file, read whole: OpenTitan's Earlgrey ASIC constraints
//! (`tests/data/sdc/opentitan/`, Apache-2.0, unmodified; see the README there). This pins what the
//! reader makes of it without a netlist, as structured results.
//!
//! The file is one part of a synthesis flow, and reads only as that flow reads it:
//! - the flow defines its variables first (`CLK_PERIOD_FACTOR`, the clock pins, the pad cells,
//!   `FOUNDRY_ROOT`, `IS_CDC_RUN`), which [`ReadOpts::defines`] supplies here, with placeholder
//!   pin and cell names;
//! - it calls one helper the flow defines, `gpo`, which is no SDC command and no command of the
//!   reference timer: read alone, the file stops there, as the reference would ([`as_published`]).
//!   The flow's definition is not public, so a stand-in `proc gpo` (the pins it names) is read
//!   first.
//!
//! The file tests `synopsys_program_name` and sets it to `""` when it does not exist. Nothing here
//! defines it — `info exists` looks at the file's variables, not the environment — so the read
//! takes the file's own default and the non-`pt_shell` branches, the same on every machine.
//!
//! `set_sense` appears six times, each `-stop_propagation` with no `-type`: four are read as clock
//! senses (the SDC 2.1 default) and reported as such; the two whose objects use
//! `get_pins -leaf` are refused, as the reference refuses `-leaf`.

use vyges_loom::sdc::{ReadOpts, Sdc};
use vyges_loom::sdc_lint::{lint_without_design, Severity};

const FILE: &str = "tests/data/sdc/opentitan/chip_earlgrey_asic.sdc";

/// The flow's variables: every one the file uses and does not set, plus those it sets from
/// themselves (`set CLK_PERIOD_FACTOR $CLK_PERIOD_FACTOR`). Pin and cell names are placeholders.
const DEFINES: [(&str, &str); 15] = [
    ("CLK_PERIOD_FACTOR", "1"),
    ("MAIN_TCK_FACTOR", "1"),
    ("IS_CDC_RUN", "0"),
    ("FOUNDRY_ROOT", ""),
    ("MAIN_CLK_PIN", "u_ast/clk_main_o"),
    ("USB_CLK_PIN", "u_ast/clk_usb_o"),
    ("IO_CLK_PIN", "u_ast/clk_io_o"),
    ("AON_CLK_PIN", "u_ast/clk_aon_o"),
    ("CLK_PIN", "Y"),
    ("CLK_DST_PIN", "Q"),
    ("DRIVING_PAD", "PADCELL"),
    ("DRIVING_PAD_PIN", "PAD"),
    ("LOAD_PAD", "PADCELL"),
    ("LOAD_PAD_LIB", "padlib"),
    ("LOAD_PAD_PIN", "PAD"),
];

/// The stand-in for the flow's `gpo` helper.
const GPO: &str = "proc gpo {name} { return [get_pins $name] }\n";

fn opts() -> ReadOpts {
    ReadOpts {
        defines: DEFINES.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        base_dir: Some("tests/data/sdc/opentitan".into()),
    }
}

fn text() -> String {
    std::fs::read_to_string(FILE).unwrap_or_else(|e| panic!("{FILE}: {e}"))
}

fn read() -> Sdc {
    Sdc::parse_opts(&format!("{GPO}{}", text()), &opts()).unwrap_or_else(|e| panic!("{FILE}: {}", e.0))
}

/// Rule: a command the reference does not know stops the read (the reference stops the file
/// there). Read without the flow's helper, the file stops at its one `gpo` call.
#[test]
fn as_published() {
    let e = Sdc::parse_opts(&text(), &opts()).expect_err("gpo is defined by the flow, not here");
    assert!(e.0.starts_with("invalid command name \"gpo\" in `set_disable_clock_gating_check"), "{}", e.0);
}

/// Rule (Tcl, so the reference): reading an undefined variable stops the read. Without the
/// flow's variables the file stops at its first use of one, `set CLK_PERIOD_FACTOR
/// $CLK_PERIOD_FACTOR`, and no clock is made on a pin named after the variable.
#[test]
fn without_the_flows_variables_it_stops_at_the_first() {
    let e = Sdc::parse(&format!("{GPO}{}", text())).expect_err("CLK_PERIOD_FACTOR is the flow's");
    assert!(e.0.starts_with("can't read \"CLK_PERIOD_FACTOR\": no such variable in `set CLK_PERIOD_FACTOR $CLK_PERIOD_FACTOR"), "{}", e.0);
}

#[test]
fn reads_to_the_end_with_its_clocks() {
    let s = read();
    assert_eq!(s.clocks.len(), 34);
    assert_eq!(s.clocks.iter().filter(|c| c.generated.is_some()).count(), 19);
    let main: Vec<(&str, &[String], f64)> =
        s.clocks.iter().take(3).map(|c| (c.name.as_str(), c.sources.as_slice(), c.period)).collect();
    assert_eq!(
        main,
        [
            ("MAIN_CLK", &["u_ast/clk_main_o".to_string()][..], 10.0),
            ("USB_CLK", &["u_ast/clk_usb_o".to_string()][..], 20.8),
            ("IO_CLK", &["u_ast/clk_io_o".to_string()][..], 10.416),
        ]
    );
    assert_eq!((s.input_delays.len(), s.output_delays.len()), (24, 16));
    assert_eq!((s.exceptions.len(), s.clock_groups.len(), s.case_values.len()), (61, 3, 65));
}

/// `synopsys_program_name` is not defined, so the file's default `""` holds: the uncertainty is
/// the non-`pt_shell` 0.5, not the `pt_shell` 0.05.
#[test]
fn the_tool_name_takes_the_files_default() {
    let s = read();
    let main = s.uncertainties.iter().find(|u| u.objects == ["MAIN_CLK"]).expect("MAIN_CLK uncertainty");
    assert_eq!(main.value, 0.5);
}

/// The six `set_sense -stop_propagation`, none with `-type`: four read as clock senses (SDC 2.1),
/// kept and reported; two refused for `get_pins -leaf` in their objects.
#[test]
fn its_senses_without_a_type_are_read_as_clock() {
    let s = read();
    let senses: Vec<(&str, usize, &[String], &str)> =
        s.senses.iter().map(|c| (c.cmd.as_str(), c.keys.len(), c.flags.as_slice(), c.objects[0].as_str())).collect();
    let stop = &["-stop_propagation".to_string()][..];
    assert_eq!(
        senses,
        [
            ("set_sense", 0, stop, "top_earlgrey/earlgrey_pd_main/u_spi_device/u_reg/u_control_mode/q_reg*/Q"),
            ("set_sense", 0, stop, "top_earlgrey/earlgrey_pd_aon/u_pinmux/dio_pad_attr_q_reg_*__invert/Q"),
            ("set_sense", 0, stop, "top_earlgrey/earlgrey_pd_aon/u_pinmux/dio_out_retreg_q_reg*/Q"),
            ("set_sense", 0, stop, "top_earlgrey/earlgrey_pd_aon/u_pinmux/u_reg/u_dio_pad_sleep_status_en*/q_reg*/Q"),
        ]
    );
    assert_eq!(s.senses_type_defaulted, [0, 1, 2, 3]);
    let refused: Vec<&String> = s.rejected.iter().filter(|r| r.starts_with("set_sense:")).collect();
    assert_eq!(refused, ["set_sense: `get_pins -leaf`: -leaf is not an option of get_pins; not applied"; 2]);
    let w = s.write(&vyges_loom::sdc_write::WriteOpts::default()).text;
    assert_eq!(w.matches("set_sense -type clock -stop_propagation ").count(), 4, "{w}");
}

/// What check reports for the file, by code and severity, with its counts.
#[test]
fn check_reports_its_findings() {
    let r = lint_without_design(&read());
    let got: Vec<(Severity, &str, String)> = r
        .findings
        .iter()
        .filter(|f| f.code != "sdc-not-applied")
        .map(|f| (f.severity, f.code, f.message.clone()))
        .collect();
    let want: Vec<(Severity, &str, &str)> = vec![
        (Severity::Warning, "sdc-kept-not-applied", "`set_ideal_network`: kept and written back; it can change timing, and no checker or timer here applies it"),
        (Severity::Warning, "sdc-kept-not-applied", "`set_sense` (4 commands): kept and written back; it can change timing, and no checker or timer here applies it"),
        (Severity::Info, "design-checks-not-run", "no netlist: I/O coverage, clock reach, exception reachability and endpoint coverage were not checked"),
        // bare name lists (`-from {SPI_HOST_D*}`): their kind is the design's to resolve
        (Severity::Info, "exception-overlap-not-checked", "13 exception(s) name objects by a query or bare name the overlap check cannot classify, and were not checked for overlap (one of them: `set_false_path -from {SPI_DEV_CS_L} -to {SPI_DEV_D*}`)"),
        (Severity::Info, "sdc-extension", "`set_clock_sense -clock` (7 uses): a reference-timer extension no SDC version declares"),
        (Severity::Info, "sdc-extension", "`set_disable_clock_gating_check`: a reference-timer extension no SDC version declares"),
        (Severity::Info, "sdc-sense-type-default", "`set_sense` (4 commands) no -type: applied as clock (SDC 2.1 default); the reference timer ignores it without -type"),
    ];
    let want: Vec<(Severity, &str, String)> = want.into_iter().map(|(s, c, m)| (s, c, m.to_string())).collect();
    assert_eq!(got, want);
    // not applied: (severity, command, count) — the queries that need the design, `get_pins
    // -leaf` (not a reference option), a `[command]` joined to text, `load_of`, `get_cells -h`
    let na: Vec<(Severity, String)> = r
        .findings
        .iter()
        .filter(|f| f.code == "sdc-not-applied")
        .map(|f| (f.severity, f.message.split(':').next().unwrap_or("").to_string()))
        .collect();
    let e = Severity::Error;
    let want_na: Vec<(Severity, &str)> = vec![
        (e, "`set_clock_sense` (11 commands)"),
        (e, "`set_false_path` (25 commands)"),
        (e, "`set_multicycle_path` (6 commands)"),
        (e, "`set_output_delay` (6 commands)"),
        (e, "`set_false_path` (47 commands)"),
        (e, "`set_false_path`"),
        (e, "`set_false_path`"),
        (e, "`set_sense` (2 commands)"),
        (e, "`set_disable_clock_gating_check`"),
        (e, "`set_load`"),
        (Severity::Info, "`set_dont_touch`"),
        (Severity::Info, "`set_size_only`"),
    ];
    let want_na: Vec<(Severity, String)> = want_na.into_iter().map(|(s, m)| (s, m.to_string())).collect();
    assert_eq!(na, want_na);
}
