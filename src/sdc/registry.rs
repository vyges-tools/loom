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
    /// Recognised and skipped: it steers synthesis or power, not timing. `write` lists it as not
    /// written.
    ReadPast,
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
            Status::Unhandled => "Unhandled",
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
    /// Whether it is one of the SDC command set ([`SDC_COMMANDS`]) rather than a command other
    /// tools' constraint files use ([`OTHER_COMMANDS`]).
    pub sdc: bool,
    /// One line that uses it, read after [`PROBE_SETUP`] by the registry's test.
    pub probe: &'static str,
}

const R: &[&str] = &["reader"];
const CHECK_STA: &[&str] = &["check", "sta", "sta-si"];
const STA: &[&str] = &["sta", "sta-si"];
const STA_SI: &[&str] = &["sta-si"];
const RSZ: &[&str] = &["rsz"];
const NONE: &[&str] = &[];

const fn cmd(name: &'static str, status: Status, applied_by: &'static [&'static str], probe: &'static str) -> Command {
    Command { name, spelling_of: None, status, applied_by, sdc: true, probe }
}
const fn spelling(name: &'static str, of: &'static str, probe: &'static str) -> Command {
    Command { name, spelling_of: Some(of), status: Status::Applied, applied_by: R, sdc: true, probe }
}
const fn kept(name: &'static str, probe: &'static str) -> Command {
    Command { name, spelling_of: None, status: Status::Kept, applied_by: NONE, sdc: true, probe }
}
const fn other(c: Command) -> Command {
    Command { sdc: false, ..c }
}

/// What every probe is read after: a clock, so a probe can name one.
pub const PROBE_SETUP: &str = "create_clock -name c -period 10 [get_ports clk]\n";

use Status::{Applied, ReadPast, Tcl, Unhandled};

/// The SDC command set: every command the SDC versions declare, in name order.
pub const SDC_COMMANDS: &[Command] = &[
    cmd("all_clocks", Applied, R, "set_propagated_clock [all_clocks]"),
    cmd("all_inputs", Applied, R, "set_input_delay 1 -clock c [all_inputs]"),
    cmd("all_outputs", Applied, R, "set_output_delay 1 -clock c [all_outputs]"),
    cmd("all_registers", Applied, R, "set_false_path -to [all_registers]"),
    cmd("create_clock", Applied, CHECK_STA, "create_clock -name d -period 5 [get_ports clk2]"),
    cmd("create_generated_clock", Applied, CHECK_STA, "create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins r/Q]"),
    kept("create_voltage_area", "create_voltage_area -name va -coordinate {0 0 10 10} [get_cells u1]"),
    cmd("current_design", Applied, R, "current_design top"),
    cmd("current_instance", Applied, R, "current_instance"),
    cmd("expr", Tcl, NONE, "set x [expr 2 * 3]"),
    spelling("get_cell", "get_cells", "set_false_path -through [get_cell u1]"),
    cmd("get_cells", Applied, R, "set_false_path -through [get_cells u1]"),
    spelling("get_clock", "get_clocks", "set_clock_uncertainty 0.1 [get_clock c]"),
    cmd("get_clocks", Applied, R, "set_clock_uncertainty 0.1 [get_clocks c]"),
    spelling("get_lib_cell", "get_lib_cells", "set_disable_timing [get_lib_cell lib/BUF]"),
    cmd("get_lib_cells", Applied, R, "set_disable_timing [get_lib_cells lib/BUF]"),
    spelling("get_lib_pin", "get_lib_pins", "set_disable_timing [get_lib_pin lib/BUF/A]"),
    cmd("get_lib_pins", Applied, R, "set_disable_timing [get_lib_pins lib/BUF/A]"),
    cmd("get_libs", Applied, R, "set_driving_cell -lib_cell BUF -library [get_libs lib] [get_ports a]"),
    spelling("get_net", "get_nets", "set_load 0.1 [get_net n1]"),
    cmd("get_nets", Applied, R, "set_load 0.1 [get_nets n1]"),
    spelling("get_pin", "get_pins", "set_false_path -to [get_pin r/D]"),
    cmd("get_pins", Applied, R, "set_false_path -to [get_pins r/D]"),
    spelling("get_port", "get_ports", "set_input_delay 1 -clock c [get_port a]"),
    cmd("get_ports", Applied, R, "set_input_delay 1 -clock c [get_ports a]"),
    kept("group_path", "group_path -name g -to [get_ports y]"),
    cmd("list", Tcl, NONE, "set_false_path -to [list [get_ports y]]"),
    cmd("set", Tcl, NONE, "set p 10"),
    cmd("set_case_analysis", Applied, STA, "set_case_analysis 0 [get_ports a]"),
    kept("set_clock_gating_check", "set_clock_gating_check -setup 0.1 [get_clocks c]"),
    Command { name: "set_clock_group", spelling_of: Some("set_clock_groups"), status: Applied, applied_by: STA, sdc: true, probe: "set_clock_group -asynchronous -group c" },
    cmd("set_clock_groups", Applied, STA, "set_clock_groups -asynchronous -group c"),
    cmd("set_clock_latency", Applied, STA, "set_clock_latency 0.5 [get_clocks c]"),
    kept("set_clock_sense", "set_clock_sense -positive [get_pins u1/Z]"),
    cmd("set_clock_transition", Applied, STA, "set_clock_transition 0.1 [get_clocks c]"),
    cmd("set_clock_uncertainty", Applied, STA, "set_clock_uncertainty 0.2 [get_clocks c]"),
    cmd("set_data_check", Applied, STA_SI, "set_data_check -from [get_pins r/D] -to [get_pins s/D] 0.1"),
    cmd("set_disable_timing", Applied, STA_SI, "set_disable_timing [get_cells u1]"),
    kept("set_drive", "set_drive 1 [get_ports a]"),
    cmd("set_driving_cell", Applied, STA, "set_driving_cell -lib_cell BUF [get_ports a]"),
    cmd("set_false_path", Applied, CHECK_STA, "set_false_path -from [get_ports a]"),
    kept("set_fanout_load", "set_fanout_load 2 [get_ports y]"),
    cmd("set_hierarchy_separator", Applied, R, "set_hierarchy_separator /"),
    kept("set_ideal_latency", "set_ideal_latency 0.1 [get_pins r/CK]"),
    kept("set_ideal_network", "set_ideal_network [get_ports clk]"),
    kept("set_ideal_transition", "set_ideal_transition 0.1 [get_ports clk]"),
    cmd("set_input_delay", Applied, CHECK_STA, "set_input_delay 1 -clock c [get_ports a]"),
    cmd("set_input_transition", Applied, STA, "set_input_transition 0.1 [get_ports a]"),
    kept("set_level_shifter_strategy", "set_level_shifter_strategy -rule all"),
    kept("set_level_shifter_threshold", "set_level_shifter_threshold -voltage 0.1"),
    cmd("set_load", Applied, STA, "set_load 0.1 [get_ports y]"),
    cmd("set_logic_dc", Applied, STA, "set_logic_dc [get_ports a]"),
    cmd("set_logic_one", Applied, STA, "set_logic_one [get_ports a]"),
    cmd("set_logic_zero", Applied, STA, "set_logic_zero [get_ports a]"),
    kept("set_max_area", "set_max_area 0"),
    cmd("set_max_capacitance", Applied, RSZ, "set_max_capacitance 0.2 [current_design]"),
    cmd("set_max_delay", Applied, &["check", "sta"], "set_max_delay 5 -to [get_ports y]"),
    kept("set_max_dynamic_power", "set_max_dynamic_power 1"),
    cmd("set_max_fanout", Applied, RSZ, "set_max_fanout 8 [current_design]"),
    kept("set_max_leakage_power", "set_max_leakage_power 1"),
    kept("set_max_time_borrow", "set_max_time_borrow 0.5 [get_clocks c]"),
    cmd("set_max_transition", Applied, RSZ, "set_max_transition 0.5 [current_design]"),
    kept("set_min_capacitance", "set_min_capacitance 0.01 [get_ports a]"),
    cmd("set_min_delay", Applied, &["check", "sta"], "set_min_delay 0.1 -to [get_ports y]"),
    cmd("set_min_porosity", ReadPast, NONE, "set_min_porosity 10 [get_cells u1]"),
    kept("set_min_pulse_width", "set_min_pulse_width 1 [get_clocks c]"),
    cmd("set_multicycle_path", Applied, CHECK_STA, "set_multicycle_path 2 -to [get_ports y]"),
    kept("set_operating_conditions", "set_operating_conditions typ"),
    cmd("set_output_delay", Applied, CHECK_STA, "set_output_delay 1 -clock c [get_ports y]"),
    kept("set_port_fanout_number", "set_port_fanout_number 2 [get_ports y]"),
    cmd("set_propagated_clock", Applied, STA, "set_propagated_clock [get_clocks c]"),
    kept("set_resistance", "set_resistance 1 [get_nets n1]"),
    kept("set_sense", "set_sense -type clock -positive [get_pins u1/Z]"),
    cmd("set_timing_derate", Applied, STA, "set_timing_derate -late 1.05"),
    Command { name: "set_unit", spelling_of: Some("set_units"), status: Applied, applied_by: R, sdc: true, probe: "set_unit -time ns" },
    cmd("set_units", Applied, R, "set_units -time ns"),
    kept("set_voltage", "set_voltage 1.0"),
    kept("set_wire_load_min_block_size", "set_wire_load_min_block_size 10"),
    kept("set_wire_load_mode", "set_wire_load_mode top"),
    kept("set_wire_load_model", "set_wire_load_model -name wl"),
    kept("set_wire_load_selection_group", "set_wire_load_selection_group sel"),
];

/// Commands found in constraint files written for other tools, outside the SDC command set, and
/// the reader's undo commands for SDC ones.
pub const OTHER_COMMANDS: &[Command] = &[
    other(cmd("remove_propagated_clock", Applied, R, "remove_propagated_clock [get_clocks c]")),
    other(cmd("set_clock_jitter", Unhandled, NONE, "set_clock_jitter -cycle 0.1 [get_clocks c]")),
    other(cmd("set_current_design", Unhandled, NONE, "set_current_design top")),
    other(kept("set_disable_clock_gating_check", "set_disable_clock_gating_check [get_cells u1]")),
    other(cmd("set_dont_touch", ReadPast, NONE, "set_dont_touch [get_cells u1]")),
    other(kept("set_path_margin", "set_path_margin 0.1 -to [get_ports y]")),
    other(kept("set_pvt", "set_pvt -voltage 1.0 [get_cells u1]")),
    other(cmd("unset_data_check", Applied, STA_SI, "unset_data_check -from [get_pins r/D] -to [get_pins s/D]")),
    other(kept("unset_disable_clock_gating_check", "unset_disable_clock_gating_check [get_cells u1]")),
    other(cmd("unset_disable_timing", Applied, STA_SI, "unset_disable_timing [get_cells u1]")),
    other(cmd("unset_propagated_clock", Applied, R, "unset_propagated_clock [get_clocks c]")),
];

/// The registry's entry for `name`, if it has one.
pub fn command(name: &str) -> Option<&'static Command> {
    SDC_COMMANDS.iter().chain(OTHER_COMMANDS).find(|c| c.name == name)
}

/// Whether `name` is read and written back but applied by no checker or timer here.
pub fn kept_not_applied(name: &str) -> bool {
    command(name).is_some_and(|c| c.status == Status::Kept)
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
                Status::Unhandled => unreachable!(),
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

    #[test]
    fn names_are_sorted_and_unique() {
        for list in [SDC_COMMANDS, OTHER_COMMANDS] {
            for w in list.windows(2) {
                assert!(w[0].name < w[1].name, "{} before {}", w[0].name, w[1].name);
            }
        }
    }
}
