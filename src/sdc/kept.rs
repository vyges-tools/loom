//! The reference's SDC commands that have no typed model here, KEPT AS WRITTEN: read by the
//! reference's own argument rules, carried in file order, and written back.
//!
//! The writer's round trip is the reference reading the original file and reading ours: a command
//! written back as it was read builds the same database, whatever the reference does with it. So
//! a command this reader cannot model still need not be lost — it is kept ([`KeptCmd`]), its
//! objects with the accessor that named them, its values as words, and written again.
//!
//! - **Exceptions** — `group_path` and `set_path_margin` are path exceptions in the reference
//!   (`Sdc::exceptions_`), so `unset_path_exceptions` and `-reset_path` reach them: they are kept
//!   as [`ExceptionCmd`], in [`Sdc::other_exceptions`], next to the false paths.
//! - **Everything else** ([`KEPT_COMMANDS`]) is a [`KeptCmd`] in [`Sdc::kept`].
//!
//! Each is still listed in [`Sdc::ignored`]: kept is not modelled, and a consumer that does not
//! model a command must keep seeing it (`MODELLED_BUT_LISTED` stops the writer and `check`
//! reporting it as dropped).
//!
//! ⛔ A command the reference REFUSES (its `sta_error`, or Tcl's wrong # args) is refused here
//! with the reference's message number, and not kept: a consumer must not read a constraint the
//! reference never made.

use super::{brace_word, exception_args, Objects, Sdc};
use crate::liberty::LibGroups;

/// The commands kept as [`KeptCmd`], each with the reference's keys (`-name value`), the keys
/// whose value names objects, its flags, and how many positionals it takes (`min`, `max`) and
/// which of them name objects. From the reference's `define_cmd_args` / `parse_key_args` and the
/// `check_argc_*` each proc runs.
struct Spec {
    cmd: &'static str,
    keys: &'static [&'static str],
    object_keys: &'static [&'static str],
    flags: &'static [&'static str],
    argc: (usize, usize),
    object_args: &'static [usize],
}

const ANY: usize = usize::MAX;

/// Procs with fixed parameters (`proc set_max_area { area }`): no option parse, so every word is
/// a parameter and a count other than theirs is Tcl's wrong # args.
const FIXED: [&str; 8] = [
    "set_max_time_borrow", "set_min_capacitance", "set_fanout_load", "set_max_area", "set_max_dynamic_power",
    "set_max_leakage_power", "set_wire_load_min_block_size", "set_wire_load_mode",
];
const DATA_CHECK_KEYS: [&str; 7] = ["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-clock"];

const SPECS: &[Spec] = &[
    Spec { cmd: "set_min_pulse_width", keys: &[], object_keys: &[], flags: &["-low", "-high"], argc: (1, 2), object_args: &[1] },
    Spec { cmd: "set_max_time_borrow", keys: &[], object_keys: &[], flags: &[], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_clock_gating_check", keys: &["-setup", "-hold"], object_keys: &[], flags: &["-rise", "-fall", "-high", "-low"], argc: (0, 1), object_args: &[0] },
    Spec { cmd: "set_data_check", keys: &DATA_CHECK_KEYS, object_keys: &DATA_CHECK_KEYS, flags: &["-setup", "-hold"], argc: (1, 1), object_args: &[] },
    Spec { cmd: "unset_data_check", keys: &DATA_CHECK_KEYS, object_keys: &DATA_CHECK_KEYS, flags: &["-setup", "-hold"], argc: (0, 0), object_args: &[] },
    Spec { cmd: "set_drive", keys: &[], object_keys: &[], flags: &["-rise", "-fall", "-min", "-max"], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_port_fanout_number", keys: &[], object_keys: &[], flags: &["-max", "-min"], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_resistance", keys: &[], object_keys: &[], flags: &["-max", "-min"], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_min_capacitance", keys: &[], object_keys: &[], flags: &[], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_fanout_load", keys: &[], object_keys: &[], flags: &[], argc: (2, 2), object_args: &[1] },
    Spec { cmd: "set_max_area", keys: &[], object_keys: &[], flags: &[], argc: (1, 1), object_args: &[] },
    Spec { cmd: "set_max_dynamic_power", keys: &[], object_keys: &[], flags: &[], argc: (1, 2), object_args: &[] },
    Spec { cmd: "set_max_leakage_power", keys: &[], object_keys: &[], flags: &[], argc: (1, 2), object_args: &[] },
    Spec {
        cmd: "set_operating_conditions",
        keys: &["-analysis_type", "-library", "-min", "-max", "-min_library", "-max_library"],
        object_keys: &[],
        flags: &[],
        argc: (0, 1),
        object_args: &[],
    },
    Spec { cmd: "set_wire_load_mode", keys: &[], object_keys: &[], flags: &[], argc: (1, 1), object_args: &[] },
    Spec { cmd: "set_wire_load_model", keys: &["-name", "-library"], object_keys: &[], flags: &["-min", "-max"], argc: (0, 1), object_args: &[0] },
    Spec { cmd: "set_wire_load_selection_group", keys: &["-library"], object_keys: &[], flags: &["-min", "-max"], argc: (1, 2), object_args: &[1] },
    Spec { cmd: "set_wire_load_min_block_size", keys: &[], object_keys: &[], flags: &[], argc: (1, 1), object_args: &[] },
    Spec { cmd: "set_voltage", keys: &["-min", "-object_list"], object_keys: &["-object_list"], flags: &[], argc: (1, 1), object_args: &[] },
    Spec { cmd: "set_pvt", keys: &["-process", "-voltage", "-temperature"], object_keys: &[], flags: &["-min", "-max"], argc: (1, 1), object_args: &[0] },
    // parsed and ignored by the reference: any arguments
    Spec { cmd: "set_ideal_latency", keys: &[], object_keys: &[], flags: &["-rise", "-fall", "-min", "-max"], argc: (0, ANY), object_args: &[1] },
    Spec { cmd: "set_ideal_network", keys: &[], object_keys: &[], flags: &["-no_propagation"], argc: (0, ANY), object_args: &[0] },
    Spec { cmd: "set_ideal_transition", keys: &[], object_keys: &[], flags: &["-rise", "-fall", "-min", "-max"], argc: (0, ANY), object_args: &[1] },
    Spec {
        cmd: "create_voltage_area",
        keys: &["-name", "-coordinate", "-guard_band_x", "-guard_band_y"],
        object_keys: &[],
        flags: &[],
        argc: (0, ANY),
        object_args: &[0],
    },
    Spec { cmd: "set_level_shifter_strategy", keys: &["-rule"], object_keys: &[], flags: &[], argc: (0, ANY), object_args: &[] },
    Spec { cmd: "set_level_shifter_threshold", keys: &["-voltage"], object_keys: &[], flags: &[], argc: (0, ANY), object_args: &[] },
];

/// The commands kept as [`KeptCmd`].
pub(super) const KEPT_COMMANDS: &[&str] = &[
    "set_min_pulse_width",
    "set_max_time_borrow",
    "set_clock_gating_check",
    "set_data_check",
    "unset_data_check",
    "set_drive",
    "set_port_fanout_number",
    "set_resistance",
    "set_min_capacitance",
    "set_fanout_load",
    "set_max_area",
    "set_max_dynamic_power",
    "set_max_leakage_power",
    "set_operating_conditions",
    "set_wire_load_mode",
    "set_wire_load_model",
    "set_wire_load_selection_group",
    "set_wire_load_min_block_size",
    "set_voltage",
    "set_pvt",
    "set_ideal_latency",
    "set_ideal_network",
    "set_ideal_transition",
    "create_voltage_area",
    "set_level_shifter_strategy",
    "set_level_shifter_threshold",
];

/// The path exceptions kept as [`ExceptionCmd`].
pub(super) const EXCEPTION_COMMANDS: &[&str] = &["group_path", "set_path_margin"];

/// One word of a kept command: a value as written (a number, a mode, a library name), or objects
/// with the accessor that named them.
#[derive(Debug, Clone, PartialEq)]
pub enum KeptArg {
    Word(String),
    Objects(Objects),
}

impl KeptArg {
    /// The value, when this is one.
    pub fn word(&self) -> Option<&str> {
        match self {
            KeptArg::Word(w) => Some(w),
            KeptArg::Objects(_) => None,
        }
    }
    /// The objects, when this names some.
    pub fn objects(&self) -> Option<&Objects> {
        match self {
            KeptArg::Objects(o) => Some(o),
            KeptArg::Word(_) => None,
        }
    }
}

/// One command kept as written — see the module documentation. Keys in the order written, each
/// with its value; flags; positionals.
#[derive(Debug, Clone, PartialEq)]
pub struct KeptCmd {
    pub cmd: String,
    pub keys: Vec<(String, KeptArg)>,
    pub flags: Vec<String>,
    pub args: Vec<KeptArg>,
}

impl KeptCmd {
    /// The LAST value of key `k` (a Tcl array keeps the last), if given.
    pub fn key(&self, k: &str) -> Option<&KeptArg> {
        self.keys.iter().rev().find(|(n, _)| n == k).map(|(_, v)| v)
    }
    pub fn has(&self, f: &str) -> bool {
        self.flags.iter().any(|x| x == f)
    }
    /// Positional `i` as a number, if it is one.
    pub fn number(&self, i: usize) -> Option<f64> {
        self.args.get(i)?.word()?.parse().ok()
    }
}

/// One `group_path` or `set_path_margin`: the reference's other path exceptions.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ExceptionCmd {
    pub cmd: String,
    /// `group_path -name`; `None` for `-default` and for `set_path_margin`.
    pub name: Option<String>,
    /// `group_path -default`: the paths go back to their default group.
    pub default: bool,
    /// `set_path_margin`'s margin; `None` for `group_path`.
    pub value: Option<f64>,
    /// `-weight` / `-critical_range`, as written (the reference reads and ignores them).
    pub keys: Vec<(String, String)>,
    /// `-from` / `-through` / `-to` and their edge forms, in order, with their objects.
    pub args: Vec<(String, Objects)>,
    /// The flags without a value (`-setup`, `-hold`, `-rise`, `-fall`, `-default`).
    pub flags: Vec<String>,
    pub comment: Option<String>,
}

/// `toks` split as the reference's `parse_key_args` splits them for `spec`: a key takes the next
/// word; a flag is one of the spec's; anything else is a positional.
fn split(toks: &[String], spec: &Spec) -> Result<KeptCmd, String> {
    let mut c = KeptCmd { cmd: toks[0].clone(), keys: Vec::new(), flags: Vec::new(), args: Vec::new() };
    let mut pos: Vec<&String> = Vec::new();
    let mut k = 1;
    while k < toks.len() {
        let t = toks[k].as_str();
        if spec.keys.contains(&t) {
            let Some(v) = toks.get(k + 1) else { return Err(format!("{t} without a value")) };
            let v = if spec.object_keys.contains(&t) { KeptArg::Objects(Objects::of(v)) } else { KeptArg::Word(v.clone()) };
            c.keys.push((t.to_string(), v));
            k += 2;
            continue;
        }
        if spec.flags.contains(&t) {
            c.flags.push(t.to_string());
        } else {
            pos.push(&toks[k]);
        }
        k += 1;
    }
    let (lo, hi) = spec.argc;
    if pos.len() < lo || pos.len() > hi {
        // the reference's check_argc_* messages; a proc with fixed parameters is Tcl's own
        let msg = match (lo, hi) {
            _ if FIXED.contains(&spec.cmd) => "wrong # args".to_string(),
            (0, 0) => "positional arguments not supported (Error 564)".to_string(),
            (0, 1) => "requires zero or one positional arguments (Error 566)".to_string(),
            (1, 1) => "requires one positional argument (Error 565)".to_string(),
            (2, 2) => "requires two positional arguments (Error 567)".to_string(),
            (1, 2) => "requires one or two positional arguments (Error 568)".to_string(),
            _ => format!("{} positional arguments (wrong # args)", pos.len()),
        };
        return Err(format!("{msg}, given {}", pos.len()));
    }
    for (i, p) in pos.iter().enumerate() {
        let names_objects = spec.object_args.contains(&i) || (hi == ANY && p.starts_with('['));
        c.args.push(if names_objects { KeptArg::Objects(Objects::of(p)) } else { KeptArg::Word((*p).clone()) });
    }
    Ok(c)
}

/// Tcl's `string is double`: a float, or an empty string.
fn is_double(w: &str) -> bool {
    w.trim().is_empty() || w.trim().parse::<f64>().is_ok()
}

/// `check_positive_float` (Error 572).
fn positive_float(what: &str, w: Option<&str>) -> Result<(), String> {
    match w {
        Some(v) if is_double(v) && v.trim().parse::<f64>().map_or(true, |x| x >= 0.0) => Ok(()),
        Some(v) => Err(format!("{what} '{v}' is not a positive float (Error 572)")),
        None => Ok(()),
    }
}

/// `check_float` (Error 571).
fn float(what: &str, w: Option<&str>) -> Result<(), String> {
    match w {
        Some(v) if !is_double(v) => Err(format!("{what} '{v}' is not a float (Error 571)")),
        _ => Ok(()),
    }
}

/// The reference's own checks on a kept command, after its arguments are split: each proc's
/// `sta_error`s, in the order it raises them.
fn check(c: &KeptCmd) -> Result<(), String> {
    let arg = |i: usize| c.args.get(i).and_then(KeptArg::word);
    let key = |k: &str| c.key(k).and_then(KeptArg::word);
    match c.cmd.as_str() {
        "set_min_pulse_width" => positive_float("min pulse width", arg(0)),
        "set_max_time_borrow" => positive_float("borrow_limit", arg(0)),
        "set_drive" | "set_resistance" => positive_float("resistance", arg(0)),
        "set_min_capacitance" => positive_float("limit", arg(0)),
        "set_max_area" => positive_float("area", arg(0)),
        "set_max_dynamic_power" | "set_max_leakage_power" => positive_float("power", arg(0)),
        "set_port_fanout_number" => match arg(0) {
            Some(v) if !v.trim().parse::<i64>().is_ok_and(|n| n >= 0) => Err(format!("fanout '{v}' is not a positive integer (Error 574)")),
            _ => Ok(()),
        },
        "set_voltage" => float("max_case_voltage", arg(0)).and_then(|()| float("-min", key("-min"))),
        "set_pvt" => ["-process", "-voltage", "-temperature"].iter().try_for_each(|k| float(k, key(k))),
        "set_clock_gating_check" => {
            if c.has("-high") && c.has("-low") {
                return Err("cannot specify both -high and -low (Error 396)".into());
            }
            if c.key("-setup").is_none() && c.key("-hold").is_none() {
                return Err("missing -setup or -hold argument (Error 397)".into());
            }
            if (c.has("-high") || c.has("-low")) && c.args.is_empty() {
                return Err("-high and -low only permitted for pins and instances (Error 398)".into());
            }
            // `set_clock_gating_check1`: an object list holding a clock is refused with -high/-low
            let clocks = |o: &Objects| matches!(o.accessor.as_str(), "get_clocks" | "all_clocks");
            let holds_clock = c.args.iter().any(|a| match a {
                KeptArg::Objects(o) => clocks(o) || o.items.iter().any(clocks),
                KeptArg::Word(_) => false,
            });
            if (c.has("-high") || c.has("-low")) && holds_clock {
                return Err("-high and -low only permitted for pins and instances (Error 399)".into());
            }
            Ok(())
        }
        "set_data_check" | "unset_data_check" => {
            let (from, to) = if c.cmd == "set_data_check" { (425, 426) } else { (427, 428) };
            if !["-from", "-rise_from", "-fall_from"].iter().any(|k| c.key(k).is_some()) {
                return Err(format!("missing -from, -rise_from or -fall_from argument (Error {from})"));
            }
            if !["-to", "-rise_to", "-fall_to"].iter().any(|k| c.key(k).is_some()) {
                return Err(format!("missing -to, -rise_to or -fall_to argument (Error {to})"));
            }
            Ok(())
        }
        "set_wire_load_mode" => match arg(0) {
            Some("top" | "enclosed" | "segmented") => Ok(()),
            _ => Err("mode must be top, enclosed or segmented (Error 478)".into()),
        },
        "set_wire_load_model" => match key("-name") {
            None => Err("no wire load model specified (Error 479)".into()),
            Some(m) => lib_lookup(c, "-library", m, |g| &g.wire_loads, 480, 480, "wire load model"),
        },
        "set_wire_load_selection_group" => match arg(0) {
            Some(g) => lib_lookup(c, "-library", g, |l| &l.wire_load_selections, 481, 481, "wire load selection group"),
            None => Ok(()),
        },
        "set_operating_conditions" => {
            match key("-analysis_type") {
                None | Some("single" | "bc_wc" | "on_chip_variation") => {}
                Some(_) => return Err("-analysis_type must be single, bc_wc or on_chip_variation (Error 476)".into()),
            }
            // the positional, then -min, then -max, each in its own library key's library
            for (name, lib_key) in [(arg(0), "-library"), (key("-min"), "-min_library"), (key("-max"), "-max_library")] {
                if let Some(n) = name {
                    lib_lookup(c, lib_key, n, |g| &g.operating_conditions, 474, 475, "operating condition")?;
                }
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

/// A library-level group looked up by name as the reference does (`parse_op_cond`,
/// `set_wire_load_model`, `set_wire_load_selection_group` in its `Sdc.tcl`): in the `lib_key`
/// library when given (`get_liberty_error`: one library, found by name), else the first library
/// that has it. Not checked when no library was read.
fn lib_lookup(
    c: &KeptCmd,
    lib_key: &str,
    name: &str,
    names: fn(&LibGroups) -> &Vec<String>,
    err_in_lib: u32,
    err_any: u32,
    what: &str,
) -> Result<(), String> {
    let lib_arg: Option<Vec<String>> = c.key(lib_key).map(|a| match a {
        KeptArg::Word(w) if w.starts_with('[') => super::resolve_objs(w),
        KeptArg::Word(w) => w.split_whitespace().map(String::from).collect(),
        KeptArg::Objects(o) => o.names.clone(),
    });
    super::with_design(|d| {
        if d.libs.is_empty() {
            return Ok(());
        }
        match &lib_arg {
            Some(libs) => {
                if libs.len() > 1 {
                    return Err("library must be a single library (Error 113)".to_string());
                }
                let l = libs.first().map_or("", String::as_str);
                let Some(lib) = d.libs.iter().find(|g| g.name == l) else {
                    return Err(format!("library '{l}' not found (Error 115)"));
                };
                if names(lib).iter().any(|n| n == name) {
                    Ok(())
                } else {
                    Err(format!("{what} '{name}' not found (Error {err_in_lib})"))
                }
            }
            None if d.libs.iter().any(|g| names(g).iter().any(|n| n == name)) => Ok(()),
            None => Err(format!("{what} '{name}' not found (Error {err_any})")),
        }
    })
    .unwrap_or(Ok(()))
}

/// `toks` read as the reference reads the command, or why the reference refuses it.
pub(super) fn kept_cmd(toks: &[String]) -> Result<KeptCmd, String> {
    let spec = SPECS.iter().find(|s| s.cmd == toks[0]).ok_or_else(|| format!("{} is not kept", toks[0]))?;
    let c = split(toks, spec)?;
    check(&c)?;
    Ok(c)
}

/// `group_path` / `set_path_margin` read as the reference reads them (`group_path`,
/// `set_path_margin` in its SDC Tcl), or why it refuses the command.
pub(super) fn exception_cmd(toks: &[String]) -> Result<ExceptionCmd, String> {
    const NAMED: [&str; 3] = ["-name", "-weight", "-critical_range"];
    // the keys only group_path has are taken out first; the rest reads as any exception
    let mut rest: Vec<String> = Vec::with_capacity(toks.len());
    let mut keys: Vec<(String, String)> = Vec::new();
    let mut name = None;
    let mut k = 0;
    while k < toks.len() {
        let t = toks[k].as_str();
        if toks[0] == "group_path" && NAMED.contains(&t) {
            let Some(v) = toks.get(k + 1) else { return Err(format!("{t} without a value")) };
            if t == "-name" {
                name = Some(v.clone());
            } else {
                keys.push((t.to_string(), v.clone()));
            }
            k += 2;
            continue;
        }
        rest.push(toks[k].clone());
        k += 1;
    }
    let (args, mut flags) = exception_args(&rest);
    let (_, _, pos) = super::split_args(&rest, &super::EXC_VALUED);
    let comment = super::comment_of(toks);
    let none = !args.iter().any(|(k, _)| k.contains("from") || k.contains("through") || k.contains("to"));
    let mut x = ExceptionCmd { cmd: toks[0].clone(), name: None, default: false, value: None, keys, args, flags: Vec::new(), comment };
    if toks[0] == "group_path" {
        if !pos.is_empty() {
            return Err("positional arguments not supported (Error 392)".into());
        }
        if none {
            return Err("-from, -through or -to required (Error 393)".into());
        }
        let default = flags.iter().any(|f| f == "-default");
        match (default, name) {
            (true, Some(_)) => return Err("-name and -default are mutually exclusive (Error 394)".into()),
            (false, None) => return Err("-name or -default option is required (Error 395)".into()),
            (d, n) => {
                x.default = d;
                x.name = n;
            }
        }
    } else {
        let Some(v) = pos.first() else { return Err("missing margin argument (Error 1800)".into()) };
        if none {
            return Err("-from, -through or -to required (Error 1802)".into());
        }
        if !is_double(v) {
            return Err(format!("set_path_margin margin '{v}' is not a float (Error 571)"));
        }
        x.value = Some(v.trim().parse().unwrap_or(0.0));
    }
    flags.retain(|f| f != "-default");
    x.flags = flags;
    Ok(x)
}

/// A kept command as SDC: `<cmd> <keys> <flags> <positionals>`, or `None` when an object name
/// cannot be written.
pub(crate) fn write_kept(c: &KeptCmd, objects: impl Fn(&Objects) -> Option<String>) -> Option<String> {
    let arg = |a: &KeptArg| -> Option<String> {
        match a {
            KeptArg::Word(w) => Some(brace_word(w)),
            KeptArg::Objects(o) => objects(o),
        }
    };
    let mut t = c.cmd.clone();
    for (k, v) in &c.keys {
        t += &format!(" {k} {}", arg(v)?);
    }
    for f in &c.flags {
        t += &format!(" {f}");
    }
    for a in &c.args {
        t += &format!(" {}", arg(a)?);
    }
    Some(t)
}

impl Sdc {
    /// What `delete_clock` takes from the kept commands (`Sdc::removeClock`):
    /// `deleteLatchBorrowLimitsReferencing` and `deleteMinPulseWidthReferencing` — a clock's
    /// `set_max_time_borrow` and `set_min_pulse_width` go with it. A command left naming nothing
    /// goes.
    pub(super) fn remove_kept_clock(&mut self, name: &str) {
        self.kept.retain_mut(|c| {
            if !matches!(c.cmd.as_str(), "set_max_time_borrow" | "set_min_pulse_width") {
                return true;
            }
            let Some(KeptArg::Objects(o)) = c.args.get_mut(1) else { return true };
            let before = o.names.len();
            let clocks = o.accessor == "get_clocks" || o.accessor == "all_clocks";
            if clocks || o.accessor.is_empty() {
                o.names.retain(|n| n != name);
            }
            before == o.names.len() || !o.names.is_empty()
        });
    }
}

#[cfg(test)]
mod tests {
    use super::super::Sdc;

    fn read(t: &str) -> Sdc {
        Sdc::parse(t).expect("reads")
    }

    /// Each kept command is carried with its words, and written back as read.
    #[test]
    fn kept_commands_are_carried() {
        let s = read("set_min_pulse_width -high 0.5 [get_clocks clk]\nset_max_time_borrow 0.2 [get_pins r1/D]\nset_drive -rise 10 [get_ports in1]\nset_wire_load_mode top\n");
        let cmds: Vec<&str> = s.kept.iter().map(|c| c.cmd.as_str()).collect();
        assert_eq!(cmds, ["set_min_pulse_width", "set_max_time_borrow", "set_drive", "set_wire_load_mode"]);
        assert_eq!(s.kept[0].number(0), Some(0.5));
        assert!(s.kept[0].has("-high"));
        assert_eq!(s.kept[1].args[1].objects().map(|o| o.accessor.as_str()), Some("get_pins"));
        let w = s.write(&Default::default());
        assert!(w.text.contains("set_min_pulse_width -high 0.5 [get_clocks {clk}]"), "{}", w.text);
        assert!(w.text.contains("set_wire_load_mode top"), "{}", w.text);
        assert!(w.not_written.is_empty(), "{:?}", w.not_written);
        assert!(s.ignored.iter().any(|c| c == "set_drive"), "kept is still listed for a consumer that does not model it");
    }

    /// Rule (each proc's checks): the reference refuses these, so they are not kept.
    #[test]
    fn kept_commands_refused_as_the_reference_refuses() {
        for (cmd, msg) in [
            ("set_min_pulse_width -1 [get_clocks clk]", "572"),
            ("set_max_time_borrow 0.2", "wrong # args"),
            ("set_port_fanout_number 2.5 [get_ports a]", "574"),
            ("set_clock_gating_check -rise [get_clocks clk]", "397"),
            ("set_clock_gating_check -setup 0.1 -high -low [get_pins g/A]", "396"),
            ("set_clock_gating_check -setup 0.1 -high", "398"),
            ("set_data_check -to [get_pins r/D] 0.1", "425"),
            ("set_data_check -from [get_pins r/CK] 0.1", "426"),
            ("set_wire_load_mode flat", "478"),
            ("set_wire_load_model -library lib", "479"),
            ("set_operating_conditions -analysis_type ocv", "476"),
            ("group_path -from [get_ports a]", "395"),
            ("group_path -name g -default -from [get_ports a]", "394"),
            ("group_path -name g", "393"),
            ("set_path_margin -from [get_ports a]", "1800"),
        ] {
            let s = read(&format!("{cmd}\n"));
            assert!(s.kept.is_empty() && s.other_exceptions.is_empty(), "{cmd} was kept");
            assert!(s.rejected.iter().any(|r| r.contains(msg)), "{cmd}: {:?}", s.rejected);
        }
    }

    /// Rule (`Sdc::removeClock`): a clock's latch borrow limits and minimum pulse widths go with it.
    #[test]
    fn delete_clock_takes_borrow_and_pulse_width() {
        let s = read("create_clock -name a -period 1 [get_ports a]\ncreate_clock -name b -period 1 [get_ports b]\nset_max_time_borrow 0.2 [get_clocks a]\nset_min_pulse_width 0.3 [get_clocks {a b}]\nset_max_time_borrow 0.1 [get_pins r/D]\ndelete_clock [get_clocks a]\n");
        let left: Vec<(String, Vec<String>)> = s.kept.iter().map(|c| (c.cmd.clone(), c.args[1].objects().map(|o| o.names.clone()).unwrap_or_default())).collect();
        assert_eq!(left, [("set_min_pulse_width".to_string(), vec!["b".to_string()]), ("set_max_time_borrow".to_string(), vec!["r/D".to_string()])]);
    }

    /// Rule (Tcl runs a substitution before its command; the reference's `create_clock` returns
    /// nothing): a clock made inside a command is made first, and the command sees an empty word
    /// — measured: the reference writes that input delay with no clock.
    #[test]
    fn nested_create_clock_runs_first() {
        let s = read("set_input_delay -clock [create_clock -name clk1 -period 10 [get_ports clk1]] 2.0 [get_ports in1]\n");
        assert_eq!(s.clocks.len(), 1, "{:?}", s.rejected);
        assert_eq!(s.input_delays.len(), 1, "{:?}", s.rejected);
        assert_eq!(s.input_delays[0].clock, None);
    }

    /// Rule (`Sdc::resetPath` over `exceptions_`): group paths and path margins are exceptions, so
    /// a reset takes them; a group path holds for both checks, so a `-setup` reset does not.
    #[test]
    fn group_path_is_an_exception_to_a_reset() {
        let s = read("group_path -name g -from [get_ports {a b}]\nunset_path_exceptions -from [get_ports a]\n");
        assert_eq!(s.other_exceptions.len(), 1);
        assert_eq!(s.other_exceptions[0].args[0].1.names, ["b"]);
        let s = read("group_path -name g -from [get_ports a]\nunset_path_exceptions -setup -from [get_ports a]\n");
        assert_eq!(s.other_exceptions.len(), 1, "min_max all is not max");
        let s = read("set_path_margin -setup -from [get_ports a] 0.2\nunset_path_exceptions -setup -from [get_ports a]\n");
        assert!(s.other_exceptions.is_empty());
        let w = read("group_path -name g -weight 2 -from [get_ports a] -through [get_pins u/Z]\n").write(&Default::default());
        assert!(w.text.contains("group_path -name g -weight 2 -from [get_ports {a}] -through [get_pins {u/Z}]"), "{}", w.text);
    }
}
