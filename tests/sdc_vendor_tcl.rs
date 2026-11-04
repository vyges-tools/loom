//! The Tcl a DFT insertion tool and a signoff timer write into constraint files, read as Tcl runs
//! it (`src/sdc/tcl.rs`): procs that `return` a value or reach their caller with `upvar`, `dict`
//! tables, `foreach_in_collection`, the tool-name test, `remove_*` / `reset_*`, abbreviations.
//!
//! The fixture is written here, in the style such files take; it is no tool's output.

use vyges_loom::sdc::{ReadOpts, Sdc};

const DFT_STYLE: &str = r#"
# Synthetic constraint file in the style a DFT insertion tool writes (loom fixture).
proc dft_tool_name {} {
  global synopsys_program_name
  if {[info exists synopsys_program_name]} { return $synopsys_program_name }
  return unknown
}

proc dft_set_defaults {} {
  upvar dft_test_period period
  upvar 1 dft_shift_period shift
  set period 20.0
  set shift [expr {$period * 2}]
}
dft_set_defaults

proc dft_pins {name} {
  switch -- [dft_tool_name] {
    pt_shell { return [get_pins $name] }
    default { return [get_pins $name] }
  }
}

set dft_clocks [dict create test_clk {period 20.0 port tck} shift_clk {period 40.0 port sclk}]
dict set dft_clocks capture_clk {period 10.0 port cclk}
dict for {name spec} $dft_clocks {
  create_clock -name $name -period [dict get $spec period] [get_ports [dict get $spec port]]
}
create_clock -name tck_main -period $dft_test_period [get_ports tck2]

foreach_in_collection p [get_pins u_ctrl/scan_en_reg/Q] {
  set_case_analysis 0 $p
}

set_false_path -from [dft_pins u_edt/lfsr_reg/CK]

proc dft_early_exit {} {
  if {1} { return }
  set_false_path -to [get_ports never]
}
dft_early_exit

set_input_delay 3.0 -clock test_clk [get_ports scan_in]
remove_input_delay [get_ports scan_in]
set_input_delay 1.0 -clock test_clk [get_ports scan_in]
set_output_delay 2.0 -clock test_clk [get_ports scan_out]
remove_output_delay [get_ports scan_out]
set_propagated_clock [get_clocks shift_clk]
remove_propagated_clock [get_clocks shift_clk]
set_max_delay 5.0 -to [get_attr [get_pins u_x/A] full_name]
create_clock -name last_clk -period 7.0 [get_ports lclk]
"#;

/// The rules, each from Tcl 8.6 or the timer the file was written for:
/// - `upvar` names the caller's variable: `dft_set_defaults` sets the caller's two periods;
/// - a proc called in `[…]` is its `return` value; a `return` ends the proc (`dft_early_exit`);
/// - `dict create` / `dict set` / `dict for` / `dict get`, in insertion order;
/// - `foreach_in_collection` whose body hands `$p` to a constraint as its object is that
///   constraint on the whole collection;
/// - `remove_input_delay` / `remove_output_delay` / `remove_propagated_clock` are the reference's
///   `unset_*`, EXCEPT that a signoff timer's `remove_*_delay` with no `-clock` takes back every
///   delay on the objects, whatever its clock — the reference's `unset_*_delay` with no `-clock`
///   takes back only the delay naming no clock — so the reader issues one take-back per clock;
/// - `get_attr` is `get_attribute` (refused by name, the read going on).
#[test]
fn a_dft_tool_style_file_reads_to_its_end() {
    let s = Sdc::parse(DFT_STYLE).expect("the whole file reads");
    let clock = |n: &str| s.clocks.iter().find(|c| c.name == n).unwrap_or_else(|| panic!("clock {n}: {:?} rejected {:?}", s.clocks.iter().map(|c| &c.name).collect::<Vec<_>>(), s.rejected));
    assert_eq!(clock("test_clk").period, 20.0);
    assert_eq!(clock("shift_clk").period, 40.0);
    assert_eq!(clock("capture_clk").period, 10.0);
    assert_eq!(clock("capture_clk").sources, vec!["cclk".to_string()]);
    assert_eq!(clock("tck_main").period, 20.0, "upvar set the caller's dft_test_period");
    assert_eq!(clock("last_clk").period, 7.0, "the read reaches the end of the file");
    let order: Vec<&str> = s.clocks.iter().map(|c| c.name.as_str()).take(3).collect();
    assert_eq!(order, ["test_clk", "shift_clk", "capture_clk"], "dict for runs in insertion order");

    assert!(
        s.case_values.iter().any(|c| c.value == "0" && c.objects.iter().any(|o| o == "u_ctrl/scan_en_reg/Q")),
        "foreach_in_collection body applied to the collection: {:?}",
        s.case_values
    );
    assert!(
        s.exceptions.iter().any(|e| e.from.iter().any(|f| f.contains("lfsr_reg"))) || s.exception_objects.iter().any(|o| o[0].iter().any(|x| x.names.iter().any(|n| n.contains("lfsr_reg")))),
        "the value proc's [get_pins …] is the -from: {:?} / {:?}",
        s.exceptions,
        s.rejected
    );
    assert!(!s.exceptions.iter().any(|e| e.to.iter().any(|t| t == "never")), "dft_early_exit returned first");

    let ins: Vec<f64> = s.input_delays.iter().filter(|d| d.ports.iter().any(|p| p == "scan_in")).map(|d| d.value).collect();
    assert_eq!(ins, vec![1.0], "remove_input_delay took the 3.0 back");
    assert!(!s.output_delays.iter().any(|d| d.ports.iter().any(|p| p == "scan_out")), "remove_output_delay");
    assert!(!s.propagated.iter().any(|o| o.names.iter().any(|n| n == "shift_clk")), "remove_propagated_clock: {:?}", s.propagated);

    let attr: Vec<&String> = s.rejected.iter().filter(|r| r.contains("get_attribute")).collect();
    assert_eq!(attr.len(), 1, "get_attr is get_attribute, refused by name: {:?}", s.rejected);
    assert!(!s.rejected.iter().any(|r| r.contains("foreach_in_collection") || r.contains("upvar") || r.contains("dict")), "{:?}", s.rejected);
}

/// Tcl `return` at the top of a file `source`d (or of the file read) stops that file there.
#[test]
fn a_return_at_the_top_stops_the_file() {
    let s = Sdc::parse("create_clock -name a -period 1 [get_ports a]\nif {1} { return }\ncreate_clock -name b -period 1 [get_ports b]\n").unwrap();
    assert_eq!(s.clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["a"]);
}

/// A `foreach_in_collection` body that uses each object on its own — its name, here — needs the
/// objects one by one, which only the design has: refused by name, the read going on.
#[test]
fn a_collection_loop_that_names_each_object_is_refused() {
    let s = Sdc::parse(
        "foreach_in_collection p [get_ports in*] {\n  set_input_delay 1 -clock c [get_ports [get_object_name $p]_q]\n}\ncreate_clock -name z -period 1 [get_ports z]\n",
    )
    .unwrap();
    assert!(s.rejected.iter().any(|r| r.starts_with("foreach_in_collection")), "{:?}", s.rejected);
    assert!(s.input_delays.is_empty());
    assert_eq!(s.clocks.len(), 1, "the read goes on");
}

/// The tool-name test: `$synopsys_program_name` the flow does not define reads as a signoff
/// timer's name; `--define` picks another; `info exists` still sees only what is defined.
#[test]
fn the_tool_name_test_reads_a_signoff_timer_unless_defined() {
    let text = "switch $synopsys_program_name {\n  pt_shell { create_clock -name pt -period 1 [get_ports a] }\n  default { create_clock -name other -period 1 [get_ports a] }\n}\n\
                if {[info exists synopsys_program_name]} { create_clock -name defined -period 1 [get_ports b] } else { create_clock -name undefined -period 1 [get_ports b] }\n";
    let names = |s: &Sdc| s.clocks.iter().map(|c| c.name.clone()).collect::<Vec<_>>();
    assert_eq!(names(&Sdc::parse(text).unwrap()), ["pt", "undefined"]);
    let opts = ReadOpts { defines: vec![("synopsys_program_name".into(), "dc_shell".into())], ..Default::default() };
    assert_eq!(names(&Sdc::parse_opts(text, &opts).unwrap()), ["other", "defined"]);
}

/// The rest of `dict` as Tcl 8.6 has it: `lappend`, `incr`, `unset`, `exists`, `keys`, `size`,
/// `merge` (a later key wins, in the first one's place).
#[test]
fn dict_commands_follow_tcl() {
    let s = Sdc::parse(
        "set d [dict create a 1 b 2]\ndict lappend d l x y\ndict incr d a 5\ndict unset d b\n\
         set m [dict merge $d {a 9 z 3}]\n\
         create_clock -name k_[dict size $m] -period [dict get $m a] [get_ports a]\n\
         if {[dict exists $m z]} { create_clock -name has_z -period [dict get $d a] [get_ports q] }\n",
    )
    .unwrap();
    let c: Vec<(String, f64)> = s.clocks.iter().map(|c| (c.name.clone(), c.period)).collect();
    assert_eq!(c, vec![("k_3".to_string(), 9.0), ("has_z".to_string(), 6.0)], "{:?}", s.rejected);
}
