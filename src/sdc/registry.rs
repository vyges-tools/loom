//! Every SDC command this reader is asked about, what it does with each, and which of our tools
//! applies it. The SDC coverage table in the `vyges-sdc` README is generated from this registry,
//! and `vyges-sdc check` reports each command it reads whose status is [`Status::Kept`].
//!
//! The status is the reader's own behaviour, and a test reads one probe per command to hold it
//! to that ([`tests`]). [`Command::applied_by`] names the consumers that apply a command:
//!
//! - `reader`: evaluated while the file is read (an object query, the units, the current design
//!   or instance, a take-back command); what it changes is applied where the command it changes is;
//! - `check`: `vyges-sdc check` ([`crate::sdc_lint`]): clocks, I/O delays, exceptions;
//! - `sta`: the `vyges-tools/sta` timer (its `load` module's applied list);
//! - `sta-si`: the `vyges-tools/sta-si` timer (its SDC table and `job::MODELLED_HERE`);
//! - `rsz`: the `vyges-tools/rsz` design repair (the design-rule limits it repairs to).
//!
//! ⚠️ `applied_by` is a claim about those repositories: change it with the consumer, in the same
//! cycle, or the table claims what no tool does.

/// What the reader does with a command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Read into the model and applied by at least one consumer ([`Command::applied_by`]).
    Applied,
    /// Read into the model (a typed record or the words as written) and written back, but no
    /// checker or timer here applies it. `check` reports each one it reads.
    Kept,
    /// Tcl, run as Tcl runs it.
    Tcl,
    /// Recognised and skipped: a synthesis-only command the reference timer does not run. `write`
    /// lists it as not written.
    ReadPast,
    /// Recognised, but not evaluated: the command that uses it is reported as not applied (an
    /// error in `check` when it can move a slack), and the read goes on.
    NotApplied,
    /// Not read: the file stops there, as it stops the reference timer's read.
    Unhandled,
}

impl Status {
    /// The status as the coverage table names it.
    pub fn label(self) -> &'static str {
        match self {
            Status::Applied => "Applied",
            Status::Kept => "Kept",
            Status::Tcl => "Tcl",
            Status::ReadPast => "Read past",
            Status::NotApplied => "Not applied",
            Status::Unhandled => "Unhandled",
        }
    }
}

/// The SDC version a command first appears in. The authority is the reference SDC parser that
/// Synopsys published with the standard (one declaration file per version, 1.1 to 2.0): a command
/// is dated by the first file that declares it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Since {
    /// First declared by the reference parser's file for this version.
    Sdc(&'static str),
    // Sdc21 — SDC 2.1, which no public source declares; its one command (set_sense) is commented
    // out until a real file needs it, and the variant with it.
    // Sdc21,
    /// A Tcl command SDC files use; Tcl provides it, so the parser declares nothing.
    Tcl,
    /// Not declared by any SDC version the parser covers.
    Outside,
}

impl Since {
    /// The version as the coverage table writes it.
    pub fn label(self) -> &'static str {
        match self {
            Since::Sdc(v) => v,
            // Since::Sdc21 => "2.1 ‡",
            Since::Tcl => "Tcl",
            Since::Outside => "—",
        }
    }
}

/// One command.
#[derive(Debug, Clone, Copy)]
pub struct Command {
    pub name: &'static str,
    /// The plural query or command this one is a spelling of (`get_cell` of `get_cells`).
    pub spelling_of: Option<&'static str>,
    pub status: Status,
    /// The consumers that apply it (see the module documentation); empty unless
    /// [`Status::Applied`].
    pub applied_by: &'static [&'static str],
    /// The SDC version that introduced it — see [`Since`].
    pub since: Since,
    /// One line that uses it, read after [`PROBE_SETUP`] by the registry's test.
    pub probe: &'static str,
    /// A qualification of its status: what the reader does not do with it, or the one mode in
    /// which a consumer applies it (empty otherwise).
    pub note: &'static str,
}

const R: &[&str] = &["reader"];
const CHECK_STA: &[&str] = &["check", "sta", "sta-si"];
const STA: &[&str] = &["sta", "sta-si"];
const STA_SI: &[&str] = &["sta-si"];
const RSZ: &[&str] = &["rsz"];
const NONE: &[&str] = &[];

const fn cmd(name: &'static str, status: Status, applied_by: &'static [&'static str], probe: &'static str) -> Command {
    Command { name, spelling_of: None, status, applied_by, since: Since::Outside, probe, note: "" }
}
const fn spelling(name: &'static str, of: &'static str, probe: &'static str) -> Command {
    Command { name, spelling_of: Some(of), status: Status::Applied, applied_by: R, since: Since::Outside, probe, note: "" }
}
const fn kept(name: &'static str, probe: &'static str) -> Command {
    Command { name, spelling_of: None, status: Status::Kept, applied_by: NONE, since: Since::Outside, probe, note: "" }
}
/// `c`, dated: first declared in SDC `v`.
const fn v(v: &'static str, c: Command) -> Command {
    Command { since: Since::Sdc(v), ..c }
}
const fn since(since: Since, c: Command) -> Command {
    Command { since, ..c }
}
/// `c`, with what the reader does not do with it.
const fn note(note: &'static str, c: Command) -> Command {
    Command { note, ..c }
}

/// What every probe is read after: a clock, so a probe can name one.
pub const PROBE_SETUP: &str = "create_clock -name c -period 10 [get_ports clk]\n";

use Status::{Applied, ReadPast, Tcl, Unhandled};

/// The SDC command set: every command the SDC versions declare, in name order.
pub const SDC_COMMANDS: &[Command] = &[
    v("1.1", cmd("all_clocks", Applied, R, "set_propagated_clock [all_clocks]")),
    v("1.1", cmd("all_inputs", Applied, R, "set_input_delay 1 -clock c [all_inputs]")),
    v("1.1", cmd("all_outputs", Applied, R, "set_output_delay 1 -clock c [all_outputs]")),
    v("1.7", cmd("all_registers", Applied, R, "set_false_path -to [all_registers]")),
    v("1.1", cmd("create_clock", Applied, CHECK_STA, "create_clock -name d -period 5 [get_ports clk2]")),
    v("1.3", cmd("create_generated_clock", Applied, CHECK_STA, "create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins r/Q]")),
    v("1.6", kept("create_voltage_area", "create_voltage_area -name va -coordinate {0 0 10 10} [get_cells u1]")),
    v("1.1", cmd("current_design", Applied, R, "current_design top")),
    v("1.1", note("`-instance` or the positional; an unqualified name under it, and `-hierarchical` under it, are refused", cmd("current_instance", Applied, R, "current_instance"))),
    since(Since::Tcl, cmd("expr", Tcl, NONE, "set x [expr 2 * 3]")),
    v("1.5", spelling("get_cell", "get_cells", "set_false_path -through [get_cell u1]")),
    v("1.1", cmd("get_cells", Applied, R, "set_false_path -through [get_cells u1]")),
    v("1.5", spelling("get_clock", "get_clocks", "set_clock_uncertainty 0.1 [get_clock c]")),
    v("1.1", cmd("get_clocks", Applied, R, "set_clock_uncertainty 0.1 [get_clocks c]")),
    v("1.5", spelling("get_lib_cell", "get_lib_cells", "set_disable_timing [get_lib_cell lib/BUF]")),
    v("1.1", cmd("get_lib_cells", Applied, R, "set_disable_timing [get_lib_cells lib/BUF]")),
    v("1.5", spelling("get_lib_pin", "get_lib_pins", "set_disable_timing [get_lib_pin lib/BUF/A]")),
    v("1.1", cmd("get_lib_pins", Applied, R, "set_disable_timing [get_lib_pins lib/BUF/A]")),
    v("1.1", cmd("get_libs", Applied, R, "set_driving_cell -lib_cell BUF -library [get_libs lib] [get_ports a]")),
    v("1.5", spelling("get_net", "get_nets", "set_load 0.1 [get_net n1]")),
    v("1.1", cmd("get_nets", Applied, R, "set_load 0.1 [get_nets n1]")),
    v("1.5", spelling("get_pin", "get_pins", "set_false_path -to [get_pin r/D]")),
    v("1.1", cmd("get_pins", Applied, R, "set_false_path -to [get_pins r/D]")),
    v("1.5", spelling("get_port", "get_ports", "set_input_delay 1 -clock c [get_port a]")),
    v("1.1", cmd("get_ports", Applied, R, "set_input_delay 1 -clock c [get_ports a]")),
    v("1.7", kept("group_path", "group_path -name g -to [get_ports y]")),
    since(Since::Tcl, note("a condition the file alone decides (`info exists`, a variable); one on a query's result needs the design and is listed as not applied", cmd("if", Tcl, NONE, "if {1} { set_load 0.1 [get_ports y] }"))),
    since(Since::Tcl, cmd("list", Tcl, NONE, "set_false_path -to [list [get_ports y]]")),
    since(Since::Tcl, cmd("set", Tcl, NONE, "set p 10")),
    v("1.1", note("`sta`: through its logic simulation, with the design", cmd("set_case_analysis", Applied, STA, "set_case_analysis 0 [get_ports a]"))),
    v("1.2", kept("set_clock_gating_check", "set_clock_gating_check -setup 0.1 [get_clocks c]")),
    v("1.7", Command { name: "set_clock_group", spelling_of: Some("set_clock_groups"), status: Applied, applied_by: STA, since: Since::Outside, probe: "set_clock_group -asynchronous -group c", note: "" }),
    v("1.7", cmd("set_clock_groups", Applied, STA, "set_clock_groups -asynchronous -group c")),
    v("1.1", note("`sta-si`: only as included in an I/O delay (`-source_latency_included`, `-network_latency_included`)", cmd("set_clock_latency", Applied, STA, "set_clock_latency 0.5 [get_clocks c]"))),
    v("1.7", kept("set_clock_sense", "set_clock_sense -positive [get_pins u1/Z]")),
    v("1.1", cmd("set_clock_transition", Applied, STA, "set_clock_transition 0.1 [get_clocks c]")),
    v("1.1", cmd("set_clock_uncertainty", Applied, STA, "set_clock_uncertainty 0.2 [get_clocks c]")),
    v("1.4", note("`-rise` / `-fall`: kept, not applied", cmd("set_data_check", Applied, STA_SI, "set_data_check -from [get_pins r/D] -to [get_pins s/D] 0.1"))),
    v("1.1", cmd("set_disable_timing", Applied, STA_SI, "set_disable_timing [get_cells u1]")),
    v("1.1", kept("set_drive", "set_drive 1 [get_ports a]")),
    v("1.1", cmd("set_driving_cell", Applied, STA, "set_driving_cell -lib_cell BUF [get_ports a]")),
    v("1.1", cmd("set_false_path", Applied, CHECK_STA, "set_false_path -from [get_ports a]")),
    v("1.1", kept("set_fanout_load", "set_fanout_load 2 [get_ports y]")),
    v("1.2", note("`/` only: another stops the read", cmd("set_hierarchy_separator", Applied, R, "set_hierarchy_separator /"))),
    v("1.7", kept("set_ideal_latency", "set_ideal_latency 0.1 [get_pins r/CK]")),
    v("1.7", note("SDC's `-no_propagate` and the reference's `-no_propagation` both read", kept("set_ideal_network", "set_ideal_network [get_ports clk]"))),
    v("1.7", kept("set_ideal_transition", "set_ideal_transition 0.1 [get_ports clk]")),
    v("1.1", cmd("set_input_delay", Applied, CHECK_STA, "set_input_delay 1 -clock c [get_ports a]")),
    v("1.1", cmd("set_input_transition", Applied, STA, "set_input_transition 0.1 [get_ports a]")),
    v("1.6", kept("set_level_shifter_strategy", "set_level_shifter_strategy -rule all")),
    v("1.6", kept("set_level_shifter_threshold", "set_level_shifter_threshold -voltage 0.1")),
    v("1.1", cmd("set_load", Applied, STA, "set_load 0.1 [get_ports y]")),
    v("1.1", note("`sta`: through its logic simulation, with the design", cmd("set_logic_dc", Applied, STA, "set_logic_dc [get_ports a]"))),
    v("1.1", note("`sta`: through its logic simulation, with the design", cmd("set_logic_one", Applied, STA, "set_logic_one [get_ports a]"))),
    v("1.1", note("`sta`: through its logic simulation, with the design", cmd("set_logic_zero", Applied, STA, "set_logic_zero [get_ports a]"))),
    v("1.1", kept("set_max_area", "set_max_area 0")),
    v("1.1", cmd("set_max_capacitance", Applied, RSZ, "set_max_capacitance 0.2 [current_design]")),
    v("1.1", cmd("set_max_delay", Applied, &["check", "sta"], "set_max_delay 5 -to [get_ports y]")),
    v("1.4", kept("set_max_dynamic_power", "set_max_dynamic_power 1")),
    v("1.1", cmd("set_max_fanout", Applied, RSZ, "set_max_fanout 8 [current_design]")),
    v("1.4", kept("set_max_leakage_power", "set_max_leakage_power 1")),
    v("1.1", kept("set_max_time_borrow", "set_max_time_borrow 0.5 [get_clocks c]")),
    v("1.1", cmd("set_max_transition", Applied, RSZ, "set_max_transition 0.5 [current_design]")),
    v("1.1", kept("set_min_capacitance", "set_min_capacitance 0.01 [get_ports a]")),
    v("1.1", cmd("set_min_delay", Applied, &["check", "sta"], "set_min_delay 0.1 -to [get_ports y]")),
    v("1.4", cmd("set_min_porosity", ReadPast, NONE, "set_min_porosity 10 [get_cells u1]")),
    v("2.0", kept("set_min_pulse_width", "set_min_pulse_width 1 [get_clocks c]")),
    v("1.1", cmd("set_multicycle_path", Applied, CHECK_STA, "set_multicycle_path 2 -to [get_ports y]")),
    v("1.1", kept("set_operating_conditions", "set_operating_conditions typ")),
    v("1.1", cmd("set_output_delay", Applied, CHECK_STA, "set_output_delay 1 -clock c [get_ports y]")),
    v("1.1", kept("set_port_fanout_number", "set_port_fanout_number 2 [get_ports y]")),
    v("1.1", cmd("set_propagated_clock", Applied, STA, "set_propagated_clock [get_clocks c]")),
    v("1.1", kept("set_resistance", "set_resistance 1 [get_nets n1]")),
    // set_sense: SDC 2.1, no public definition to verify against; restore when a real file needs it:
    // since(Since::Sdc21, kept("set_sense", "set_sense -type clock -positive [get_pins u1/Z]")),
    v("1.5", cmd("set_timing_derate", Applied, STA, "set_timing_derate -late 1.05")),
    v("1.7", Command { name: "set_unit", spelling_of: Some("set_units"), status: Applied, applied_by: R, since: Since::Outside, probe: "set_unit -time ns", note: "" }),
    v("1.7", cmd("set_units", Applied, R, "set_units -time ns")),
    v("1.8", kept("set_voltage", "set_voltage 1.0")),
    v("1.1", kept("set_wire_load_min_block_size", "set_wire_load_min_block_size 10")),
    v("1.1", kept("set_wire_load_mode", "set_wire_load_mode top")),
    v("1.1", kept("set_wire_load_model", "set_wire_load_model -name wl")),
    v("1.1", kept("set_wire_load_selection_group", "set_wire_load_selection_group sel")),
];

/// Commands found in constraint files written for other tools, outside the SDC command set: the
/// reader's undo commands for SDC ones, and a synthesis tool's collection commands (SDC declares
/// none of them).
pub const OTHER_COMMANDS: &[Command] = &[
    cmd("add_to_collection", Applied, R, "set_false_path -to [add_to_collection [get_ports y] [get_ports a]]"),
    cmd("all_fanin", Status::NotApplied, NONE, "set_false_path -to [all_fanin -to [get_ports y]]"),
    cmd("all_fanout", Status::NotApplied, NONE, "set_false_path -from [all_fanout -from [get_ports a]]"),
    cmd("append_to_collection", Applied, R, "append_to_collection v [get_ports y]"),
    cmd("delete_clock", Applied, R, "delete_clock [get_clocks c]"),
    cmd("delete_generated_clock", Applied, R, "create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins r/Q]\ndelete_generated_clock [get_clocks g]"),
    note("known, not run: it stops the read", cmd("derive_clock_uncertainty", Unhandled, NONE, "derive_clock_uncertainty")),
    note("known, not run: it stops the read", cmd("derive_pll_clocks", Unhandled, NONE, "derive_pll_clocks")),
    cmd("filter_collection", Status::NotApplied, NONE, "set_false_path -to [filter_collection [get_ports y] direction==out]"),
    cmd("foreach_in_collection", Applied, R, "foreach_in_collection p [get_ports y] { set_load 0.1 $p }"),
    cmd("get_attribute", Status::NotApplied, NONE, "set_load [get_attribute [get_ports y] capacitance] [get_ports y]"),
    cmd("get_designs", Status::NotApplied, NONE, "set_false_path -through [get_designs top]"),
    cmd("get_object_name", Applied, R, "set_false_path -to [get_object_name [get_ports y]]"),
    cmd("index_collection", Status::NotApplied, NONE, "set_false_path -to [index_collection [get_ports y] 0]"),
    cmd("load_of", Status::NotApplied, NONE, "set_load [load_of lib/BUF/A] [get_ports y]"),
    note("only as every input or output but some, the objects of an I/O delay", cmd("remove_from_collection", Applied, R, "set_input_delay 1 -clock c [remove_from_collection [all_inputs] [get_ports clk]]")),
    cmd("remove_propagated_clock", Applied, R, "remove_propagated_clock [get_clocks c]"),
    cmd("set_clock_jitter", Unhandled, NONE, "set_clock_jitter -cycle 0.1 [get_clocks c]"),
    cmd("set_current_design", Unhandled, NONE, "set_current_design top"),
    kept("set_disable_clock_gating_check", "set_disable_clock_gating_check [get_cells u1]"),
    cmd("set_dont_touch", ReadPast, NONE, "set_dont_touch [get_cells u1]"),
    kept("set_path_margin", "set_path_margin 0.1 -to [get_ports y]"),
    kept("set_pvt", "set_pvt -voltage 1.0 [get_cells u1]"),
    cmd("sizeof_collection", Status::NotApplied, NONE, "set_load [sizeof_collection [get_ports y]] [get_ports y]"),
    cmd("sort_collection", Status::NotApplied, NONE, "set_false_path -to [sort_collection [get_ports y] name]"),
    cmd("unset_case_analysis", Applied, R, "set_case_analysis 0 [get_ports a]\nunset_case_analysis [get_ports a]"),
    cmd("unset_clock_groups", Applied, R, "set_clock_groups -name g -asynchronous -group c\nunset_clock_groups -asynchronous -name g"),
    cmd("unset_clock_latency", Applied, R, "set_clock_latency 0.1 [get_clocks c]\nunset_clock_latency [get_clocks c]"),
    cmd("unset_clock_transition", Applied, R, "set_clock_transition 0.1 [get_clocks c]\nunset_clock_transition [get_clocks c]"),
    cmd("unset_clock_uncertainty", Applied, R, "set_clock_uncertainty 0.1 [get_clocks c]\nunset_clock_uncertainty [get_clocks c]"),
    cmd("unset_data_check", Applied, STA_SI, "unset_data_check -from [get_pins r/D] -to [get_pins s/D]"),
    kept("unset_disable_clock_gating_check", "unset_disable_clock_gating_check [get_cells u1]"),
    cmd("unset_disable_timing", Applied, STA_SI, "unset_disable_timing [get_cells u1]"),
    cmd("unset_input_delay", Applied, R, "set_input_delay 1 -clock c [get_ports a]\nunset_input_delay -clock c [get_ports a]"),
    cmd("unset_output_delay", Applied, R, "set_output_delay 1 -clock c [get_ports y]\nunset_output_delay -clock c [get_ports y]"),
    cmd("unset_path_exceptions", Applied, R, "set_false_path -to [get_ports y]\nunset_path_exceptions -to [get_ports y]"),
    cmd("unset_propagated_clock", Applied, R, "unset_propagated_clock [get_clocks c]"),
    cmd("unset_timing_derate", Applied, R, "set_timing_derate -late 1.05\nunset_timing_derate"),
    note("an output command, not a constraint: listed as not applied", cmd("write_sdc", Status::NotApplied, NONE, "write_sdc out.sdc")),
];

/// The registry's entry for `name`, if it has one.
pub fn command(name: &str) -> Option<&'static Command> {
    SDC_COMMANDS.iter().chain(OTHER_COMMANDS).find(|c| c.name == name)
}

/// Whether `name` is read and written back but applied by no checker or timer here.
pub fn kept_not_applied(name: &str) -> bool {
    command(name).is_some_and(|c| c.status == Status::Kept)
}

/// One option the SDC standard declares that the reference timer lacks: whether a consumer here
/// applies it ([`Status::Applied`], by the reader) or it is kept with its command
/// ([`Status::Kept`]).
pub fn standard_options() -> Vec<(&'static str, &'static str, Status)> {
    let mut out = Vec::new();
    for (cmd, keys, flags) in crate::sdc_flags::STANDARD_ONLY {
        for o in keys.iter().chain(flags.iter()) {
            let kept = crate::sdc_flags::KEPT_OPTIONS.iter().any(|(c, k)| c == cmd && k == o);
            out.push((*cmd, *o, if kept { Status::Kept } else { Status::Applied }));
        }
    }
    out
}

fn row(c: &Command, with_version: bool) -> String {
    let mut status = c.status.label().to_string();
    let mut notes: Vec<String> = Vec::new();
    if let Some(of) = c.spelling_of {
        notes.push(format!("spelling of `{of}`"));
    }
    if !c.note.is_empty() {
        notes.push(c.note.to_string());
    }
    if !notes.is_empty() {
        status += &format!(" ({})", notes.join("; "));
    }
    let by: Vec<String> = c.applied_by.iter().map(|b| format!("`{b}`")).collect();
    let by = if by.is_empty() { "—".to_string() } else { by.join(", ") };
    match with_version {
        true => format!("| `{}` | {} | {status} | {by} |\n", c.name, c.since.label()),
        false => format!("| `{}` | {status} | {by} |\n", c.name),
    }
}

/// The command coverage tables, as the `vyges-sdc` README carries them between its
/// `sdc-coverage` markers: a count line, the SDC command set, the commands outside it, and the
/// standard options the reference timer lacks. Generated from this registry, so the README can
/// say no more than the reader does.
pub fn coverage_markdown() -> String {
    let count = |st: Status| SDC_COMMANDS.iter().filter(|c| c.status == st).count();
    let mut t = format!(
        "SDC 1.1 to 2.0: {} applied, {} kept, {} Tcl, {} read past, {} unhandled, of {} commands.\n\n",
        count(Status::Applied),
        count(Status::Kept),
        count(Status::Tcl),
        count(Status::ReadPast),
        count(Status::Unhandled),
        SDC_COMMANDS.len()
    );
    t += "| Command | SDC | Status | Applied by |\n| --- | --- | --- | --- |\n";
    for c in SDC_COMMANDS {
        t += &row(c, true);
    }
    t += "\nCommands outside the SDC versions the reference parser declares, found in constraint files written for other tools (a synthesis tool's collection commands among them):\n\n";
    t += "| Command | Status | Applied by |\n| --- | --- | --- |\n";
    for c in OTHER_COMMANDS {
        t += &row(c, false);
    }
    t += "\nOptions SDC declares that the reference timer does not have. They are read, never refused:\n\n";
    t += "| Command | Option | Status |\n| --- | --- | --- |\n";
    for (cmd, o, st) in standard_options() {
        t += &format!("| `{cmd}` | `{o}` | {} |\n", st.label());
    }
    t += "\nReference-timer options no SDC version declares. They are read as the reference reads them, and `check` names each one a file uses, at info level (the file is not portable SDC):\n\n";
    t += "| Command | Option |\n| --- | --- |\n";
    for (cmd, o) in crate::sdc_flags::EXTENSION_OPTIONS {
        t += &format!("| `{cmd}` | `{o}` |\n");
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdc::{Sdc, MODELLED_BUT_LISTED};
    use crate::sdc_write::WriteOpts;

    /// Each command's status is what the reader DOES with its probe, not what the table says:
    /// Unhandled stops the read; Read past is listed as not applied and changes no timing;
    /// Kept is kept, written back, and reported as kept-not-applied; Applied and Tcl are neither
    /// refused nor reported. A command the reader reclassifies fails here, and with it the table.
    #[test]
    fn every_status_is_what_the_reader_does_with_the_command() {
        let faults: Vec<String> = SDC_COMMANDS.iter().chain(OTHER_COMMANDS).filter_map(|c| std::panic::catch_unwind(|| probe(c)).err().map(|e| {
            e.downcast_ref::<String>().cloned().unwrap_or_else(|| format!("{}: (panic)", c.name))
        })).collect();
        assert!(faults.is_empty(), "{} command(s) whose status is not what the reader does:\n{}", faults.len(), faults.join("\n"));
    }

    /// The commands kept as written ([`crate::sdc::KeptCmd`]) that a consumer applies from those
    /// words: sta-si builds its data checks from them.
    const READ_FROM_KEPT: [&str; 2] = ["set_data_check", "unset_data_check"];

    fn probe(c: &Command) {
        {
            let text = format!("{PROBE_SETUP}{}\n", c.probe);
            let read = Sdc::parse(&text);
            if c.status == Status::Unhandled {
                assert!(read.is_err(), "{}: Unhandled, but the read did not stop", c.name);
                return;
            }
            let s = read.unwrap_or_else(|e| panic!("{}: {e:?}", c.name));
            if c.status == Status::NotApplied {
                assert!(!s.not_applied().is_empty(), "{}: Not applied, yet nothing was reported", c.name);
                return;
            }
            assert!(s.rejected.is_empty(), "{}: probe refused: {:?}", c.name, s.rejected);
            let not_applied: Vec<_> = s.not_applied().into_iter().map(|n| n.command).collect();
            let kept: Vec<_> = s.kept_not_applied().into_iter().map(|k| k.command).collect();
            let name = c.spelling_of.unwrap_or(c.name);
            match c.status {
                Status::ReadPast => {
                    assert!(s.not_applied().iter().any(|n| n.command == c.name && !n.affects_timing), "{}: not read past: {not_applied:?}", c.name);
                }
                Status::Kept => {
                    assert!(kept.iter().any(|k| k == name), "{}: not reported kept-not-applied: {kept:?}", c.name);
                    assert!(not_applied.is_empty(), "{}: kept, yet listed as not applied: {not_applied:?}", c.name);
                    let w = s.write(&WriteOpts::default());
                    assert!(w.text.lines().any(|l| l.starts_with(name)), "{}: kept but not written:\n{}", c.name, w.text);
                    assert!(w.not_written.is_empty(), "{}: {:?}", c.name, w.not_written);
                }
                Status::Applied | Status::Tcl => {
                    // a command carried only as its words is applied only where a consumer reads
                    // those words: sta-si's data checks (`job.rs`, from `Sdc::kept`)
                    let verbatim = s.kept.iter().any(|k| k.cmd == name) || s.other_exceptions.iter().any(|x| x.cmd == name);
                    assert!(!verbatim || READ_FROM_KEPT.contains(&name), "{}: carried only as written, so nothing applies it", c.name);
                    assert!(not_applied.is_empty(), "{}: {not_applied:?}", c.name);
                    assert!(kept.is_empty(), "{}: applied, yet reported kept-not-applied: {kept:?}", c.name);
                }
                Status::Unhandled | Status::NotApplied => unreachable!(),
            }
            assert_eq!(c.applied_by.is_empty(), c.status != Status::Applied, "{}: applied_by is for Applied only", c.name);
        }
    }

    /// Every command the reader models but lists in `ignored` for a consumer that does not has an
    /// entry: a kept command added to the reader without one would never be reported by `check`.
    #[test]
    fn every_listed_command_has_an_entry() {
        for c in MODELLED_BUT_LISTED {
            assert!(command(c).is_some(), "{c}: in MODELLED_BUT_LISTED, not in the registry");
        }
    }

    /// The SDC set is dated, by the parser's declaration files (or as 2.1 / Tcl); the other
    /// commands are outside it. A singular query spelling is dated by its own declaration (SDC
    /// 1.5), not by its plural's (1.1).
    #[test]
    fn the_sdc_set_is_dated_and_the_rest_is_not() {
        for c in SDC_COMMANDS {
            assert!(c.since != Since::Outside, "{}: in the SDC set without a version", c.name);
        }
        for c in OTHER_COMMANDS {
            assert_eq!(c.since, Since::Outside, "{}", c.name);
        }
        for q in ["get_cell", "get_clock", "get_lib_cell", "get_lib_pin", "get_net", "get_pin", "get_port"] {
            assert_eq!(command(q).map(|c| c.since), Some(Since::Sdc("1.5")), "{q}");
        }
        assert_eq!(command("get_cells").map(|c| c.since), Some(Since::Sdc("1.1")));
    }

    /// The generated table has one row per command, per standard option and per reference
    /// extension, and its count line adds up to the command set.
    #[test]
    fn the_coverage_table_has_a_row_for_everything() {
        let t = coverage_markdown();
        let (commands, rest) = t.split_once("Options SDC declares").expect("the options table follows the commands");
        let (standard, extensions) = rest.split_once("Reference-timer options").expect("the extensions table follows");
        for c in SDC_COMMANDS.iter().chain(OTHER_COMMANDS) {
            assert_eq!(commands.lines().filter(|l| l.starts_with(&format!("| `{}` |", c.name))).count(), 1, "{}", c.name);
        }
        let options = |s: &str| s.lines().filter(|l| l.split(" | ").nth(1).is_some_and(|o| o.starts_with("`-"))).count();
        assert_eq!(options(standard), standard_options().len());
        assert_eq!(options(extensions), crate::sdc_flags::EXTENSION_OPTIONS.len());
        assert!(t.contains(&format!("of {} commands", SDC_COMMANDS.len())), "{t}");
    }

    /// An extension is the reference timer's and no SDC version's: never a standard option, every
    /// extension option one the reference timer has, and every extension command outside the SDC
    /// set, with a row of its own.
    #[test]
    fn extensions_are_outside_the_standard() {
        let standard = standard_options();
        for (c, o) in crate::sdc_flags::EXTENSION_OPTIONS {
            assert!(!standard.iter().any(|(sc, so, _)| sc == c && so == o), "{c} {o} is standard");
            let (keys, flags) = crate::sdc_flags::options(c).unwrap_or_else(|| panic!("{c}: no options"));
            assert!(keys.contains(o) || flags.contains(o), "{c} {o}: not the reference timer's");
        }
        for c in crate::sdc_flags::EXTENSION_COMMANDS {
            assert!(OTHER_COMMANDS.iter().any(|x| x.name == *c), "{c}: no row outside SDC");
            assert!(command(c).is_some_and(|x| !matches!(x.since, Since::Sdc(_))), "{c}: dated as SDC");
        }
    }

    #[test]
    fn names_are_sorted_and_unique() {
        for list in [SDC_COMMANDS, OTHER_COMMANDS] {
            for w in list.windows(2) {
                assert!(w[0].name < w[1].name, "{} before {}", w[0].name, w[1].name);
            }
        }
    }
}
