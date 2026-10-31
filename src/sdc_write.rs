//! SDC writer — [`Sdc::write`]: the constraint model back out as SDC.
//!
//! What the reader kept, written so that the reference, reading it against the same design,
//! builds the same constraint database it builds from the original file. That round trip is how
//! this is scored (`correlation/instruments/loom/sdc/`): the reference writes its database from
//! the original and from ours, and the two files must be identical.
//!
//! **Order.** The reference's own `write_sdc` order — clocks in definition order (a generated
//! clock after its master), then clock properties, I/O delays, constants, exceptions, the
//! environment and the design rules. Within one kind, commands keep FILE order: the reader keeps
//! commands, not a merged database, and the reference's merge rules (a later `set_input_delay`
//! without `-add_delay` replacing, last write winning) give the same result when the commands
//! arrive in the same order. Across kinds the reference's merges are independent.
//!
//! **Values** are written in the shortest form that reads back to the same `f64` — not rounded
//! to a digit count — and in the units the file used (the reader does not scale; `set_units` is
//! written back as declared).
//!
//! **What is not written is said.** A command the reader recognised but does not model, one it
//! refused or could not evaluate, is NOT written: it is listed in [`Written::not_written`] and
//! appears in the file as a `# not written:` comment. A consumer that needs the whole file
//! refuses on a non-empty list rather than ship constraints with a hole in them.

use crate::sdc::{ExcKind, Objects, Sdc, SdcClock};

/// What [`Sdc::write`] produced.
#[derive(Debug, Clone, Default)]
pub struct Written {
    /// The SDC text.
    pub text: String,
    /// Each command not written, with why — also in `text` as a `# not written:` comment.
    pub not_written: Vec<String>,
}

/// Options for [`Sdc::write`].
#[derive(Debug, Clone, Default)]
pub struct WriteOpts {
    /// The design the constraints apply to, written as `current_design`; `None` writes the one the
    /// file named, if it named one.
    pub design: Option<String>,
    /// The first comment line (`# <header>`); `None` writes `# SDC written by vyges-loom`.
    pub header: Option<String>,
}

/// Kinds the reader models but also lists in [`Sdc::ignored`] (for a consumer that does not model
/// them): written from their own records, so not reported as unwritten.
const MODELLED_BUT_LISTED: &[&str] = &[
    "set_clock_transition",
    "set_max_fanout",
    "set_max_capacitance",
    "set_propagated_clock",
    "set_max_delay",
    "set_min_delay",
    "set_max_transition",
    "set_case_analysis",
    "set_logic_zero",
    "set_logic_one",
    "set_logic_dc",
    "set_driving_cell",
    "set_timing_derate",
];

/// A number, in the shortest form that reads back to the same value.
fn num(v: f64) -> String {
    let s = format!("{v}");
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

/// A brace-quoted Tcl word for a list of names, or `None` when a name cannot be braced (an
/// unbalanced or escaped brace).
fn brace(names: &[String]) -> Option<String> {
    if names.iter().any(|n| n.contains(['{', '}']) || n.ends_with('\\')) {
        return None;
    }
    Some(format!("{{{}}}", names.join(" ")))
}

/// The collection names the `all_*` accessors are kept under.
fn sentinel_accessor(name: &str) -> Option<&'static str> {
    match name {
        "*INPUTS*" => Some("all_inputs"),
        "*OUTPUTS*" => Some("all_outputs"),
        "*REGS*" => Some("all_registers"),
        _ => None,
    }
}

/// An object list as written: `[get_pins -hierarchical {a/b c/d}]`, `[all_inputs -no_clocks]`,
/// `[list [get_ports {a}] [get_pins {r/CK}]]`, `[current_design]`, or a bare brace list.
fn objects(o: &Objects) -> Option<String> {
    let opts = || -> String {
        let mut t = String::new();
        for f in &o.flags {
            t += &format!(" {f}");
        }
        for (k, v) in &o.keys {
            t += &format!(" {k} {}", brace(std::slice::from_ref(v)).unwrap_or_else(|| v.clone()));
        }
        t
    };
    match o.accessor.as_str() {
        "list" | "concat" if !o.items.is_empty() => {
            let inner: Option<Vec<String>> = o.items.iter().map(objects).collect();
            Some(format!("[{} {}]", o.accessor, inner?.join(" ")))
        }
        "current_design" => Some("[current_design]".into()),
        // `[all_clocks]` is kept as the clocks it named when its command ran
        "all_clocks" => named("get_clocks", &o.names),
        a if a.starts_with("all_") && !a.ends_with("_except") => Some(format!("[{a}{}]", opts())),
        "" | "list" | "concat" => match o.names.as_slice() {
            [one] if sentinel_accessor(one).is_some() => Some(format!("[{}]", sentinel_accessor(one)?)),
            names => brace(names),
        },
        a => Some(format!("[{a}{} {}]", opts(), brace(&o.names)?)),
    }
}

/// Names of one kind under one accessor (`[get_clocks {a b}]`); a collection sentinel as its
/// `all_*` accessor.
fn named(accessor: &str, names: &[String]) -> Option<String> {
    match names {
        [one] if sentinel_accessor(one).is_some() => Some(format!("[{}]", sentinel_accessor(one)?)),
        _ => Some(format!("[{accessor} {}]", brace(names)?)),
    }
}

/// A clock's source objects: a name with a hierarchy divider is a pin, else a port.
fn clock_sources(c: &SdcClock) -> Option<String> {
    if c.sources.is_empty() {
        return Some(String::new());
    }
    let (pins, ports): (Vec<String>, Vec<String>) = c.sources.iter().cloned().partition(|s| s.contains('/'));
    Some(match (pins.is_empty(), ports.is_empty()) {
        (true, _) => format!(" {}", named("get_ports", &ports)?),
        (_, true) => format!(" {}", named("get_pins", &pins)?),
        _ => format!(" [list {} {}]", named("get_ports", &ports)?, named("get_pins", &pins)?),
    })
}

struct Out {
    text: String,
    not_written: Vec<String>,
}

impl Out {
    fn line(&mut self, l: String) {
        self.text += &l;
        self.text.push('\n');
    }
    /// `l`, or — when it could not be formed — a `# not written:` record for `what`.
    fn line_or(&mut self, l: Option<String>, what: impl FnOnce() -> String) {
        match l {
            Some(l) => self.line(l),
            None => self.skip(format!("{}: an object name that cannot be written", what())),
        }
    }
    fn skip(&mut self, why: String) {
        self.text += &format!("# not written: {why}\n");
        self.not_written.push(why);
    }
    fn section(&mut self, title: &str) {
        self.text += &format!("###############################################################################\n# {title}\n###############################################################################\n");
    }
}

impl Sdc {
    /// The constraints as SDC — see the module documentation for the order, the values and what
    /// is not written.
    pub fn write(&self, opts: &WriteOpts) -> Written {
        let mut o = Out { text: String::new(), not_written: Vec::new() };
        o.line(format!("# {}", opts.header.as_deref().unwrap_or("SDC written by vyges-loom")));
        if let Some(d) = opts.design.as_ref().or(self.current_design.as_ref()) {
            o.line(format!("current_design {d}"));
        }
        for (k, v) in &self.declared_units {
            o.line(format!("set_units {k} {v}"));
        }

        o.section("Timing Constraints");
        self.write_clocks(&mut o);
        self.write_clock_properties(&mut o);
        self.write_io_delays(&mut o);
        self.write_constants(&mut o);
        self.write_exceptions(&mut o);

        o.section("Environment");
        self.write_environment(&mut o);

        o.section("Design Rules");
        self.write_design_rules(&mut o);

        self.write_unwritten(&mut o);
        Written { text: o.text, not_written: o.not_written }
    }

    /// `create_clock` / `create_generated_clock`, in definition order.
    fn write_clocks(&self, o: &mut Out) {
        for c in &self.clocks {
            let add = if c.add { " -add" } else { "" };
            let Some(srcs) = clock_sources(c) else {
                o.skip(format!("clock {}: a source name that cannot be written", c.name));
                continue;
            };
            let name = brace(std::slice::from_ref(&c.name)).unwrap_or_else(|| c.name.clone());
            match &c.generated {
                None => {
                    let mut t = format!("create_clock -name {name}{add} -period {}", num(c.period));
                    if let Some(w) = &c.waveform {
                        if w.is_empty() {
                            o.skip(format!("clock {}: a -waveform this reader could not read", c.name));
                            continue;
                        }
                        t += &format!(" -waveform {{{}}}", w.iter().map(|e| num(*e)).collect::<Vec<_>>().join(" "));
                    }
                    o.line(t + &srcs);
                }
                Some(g) => {
                    let Some(source) = objects(&g.source) else {
                        o.skip(format!("generated clock {}: a -source that cannot be written", c.name));
                        continue;
                    };
                    let mut t = format!("create_generated_clock -name {name}{add} -source {source}");
                    if let Some(m) = &g.master_clock {
                        t += &format!(" -master_clock {}", named("get_clocks", std::slice::from_ref(m)).unwrap_or_else(|| m.clone()));
                    }
                    if g.combinational {
                        t += " -combinational";
                    }
                    if let Some(d) = g.divide_by {
                        t += &format!(" -divide_by {d}");
                    }
                    if let Some(m) = g.multiply_by {
                        t += &format!(" -multiply_by {m}");
                    }
                    if let Some(d) = g.duty_cycle {
                        t += &format!(" -duty_cycle {}", num(d));
                    }
                    if g.invert {
                        t += " -invert";
                    }
                    if !g.edges.is_empty() {
                        t += &format!(" -edges {{{}}}", g.edges.iter().map(|e| e.to_string()).collect::<Vec<_>>().join(" "));
                    }
                    if !g.edge_shift.is_empty() {
                        t += &format!(" -edge_shift {{{}}}", g.edge_shift.iter().map(|e| num(*e)).collect::<Vec<_>>().join(" "));
                    }
                    o.line(t + &srcs);
                }
            }
        }
    }

    /// Propagation, slews, latencies, uncertainties and clock groups.
    fn write_clock_properties(&self, o: &mut Out) {
        for p in &self.propagated {
            o.line_or(objects(p).map(|x| format!("set_propagated_clock {x}")), || "set_propagated_clock".into());
        }
        for e in self.clock_transitions.iter().chain(&self.clock_latencies) {
            self.write_env_cmd(o, e);
        }
        for u in &self.uncertainties {
            let mut t = String::from("set_clock_uncertainty");
            for f in &u.flags {
                t += &format!(" {f}");
            }
            let mut ok = true;
            for (k, v) in &u.written_sides {
                match objects(v) {
                    Some(x) => t += &format!(" {k} {x}"),
                    None => ok = false,
                }
            }
            t += &format!(" {}", num(u.value));
            for w in &u.written {
                match objects(w) {
                    Some(x) => t += &format!(" {x}"),
                    None => ok = false,
                }
            }
            o.line_or(ok.then_some(t), || "set_clock_uncertainty".into());
        }
        for g in &self.clock_groups {
            let mut t = String::from("set_clock_groups");
            if let Some(n) = &g.name {
                t += &format!(" -name {}", brace(std::slice::from_ref(n)).unwrap_or_else(|| n.clone()));
            }
            t += &format!(" -{}", g.kind);
            if g.allow_paths {
                t += " -allow_paths";
            }
            let mut ok = true;
            for grp in &g.groups {
                match named("get_clocks", grp) {
                    Some(x) => t += &format!(" -group {x}"),
                    None => ok = false,
                }
            }
            o.line_or(ok.then_some(t), || "set_clock_groups".into());
        }
    }

    /// `set_input_delay` / `set_output_delay`, each kind in file order.
    fn write_io_delays(&self, o: &mut Out) {
        for (cmd, list) in [("set_input_delay", &self.input_delays), ("set_output_delay", &self.output_delays)] {
            for d in list {
                if !d.value_read {
                    o.skip(format!("{cmd} on {}: a value that is not a number", d.ports.join(" ")));
                    continue;
                }
                let mut head = format!("{cmd} {}", num(d.value));
                if let Some(c) = &d.clock {
                    head += &format!(" -clock {}", named("get_clocks", std::slice::from_ref(c)).unwrap_or_else(|| c.clone()));
                }
                if let Some(p) = &d.reference_pin {
                    head += &format!(" -reference_pin {}", named("get_pins", std::slice::from_ref(p)).unwrap_or_else(|| p.clone()));
                }
                for f in &d.flags {
                    head += &format!(" {f}");
                }
                let all = if cmd == "set_input_delay" { "all_inputs" } else { "all_outputs" };
                let flags: String = d.accessor_flags.iter().map(|f| format!(" {f}")).collect();
                // `[all_inputs]` minus the named ports, as the reader recognised it
                if d.accessor.ends_with("_except") {
                    let line = brace(&d.ports).map(|p| format!("{head} [lsearch -inline -all -not -exact [{all}{flags}] {p}]"));
                    o.line_or(line, || cmd.to_string());
                    continue;
                }
                // a default (`[all_inputs]`) and named ports in one command: two commands with the
                // same flags set the same cells
                if d.default {
                    o.line(format!("{head} [{all}{flags}]"));
                }
                if !d.ports.is_empty() {
                    let acc = match d.accessor.as_str() {
                        "" | "list" | "concat" => "get_ports",
                        a if a.starts_with("all_") => "get_ports",
                        a => a,
                    };
                    o.line_or(named(acc, &d.ports).map(|p| format!("{head} {p}")), || cmd.to_string());
                }
            }
        }
    }

    /// `set_case_analysis` and `set_logic_*`.
    fn write_constants(&self, o: &mut Out) {
        for c in self.case_values.iter().chain(&self.logic_values) {
            let value = if c.cmd == "set_case_analysis" { format!(" {}", c.value) } else { String::new() };
            o.line_or(objects(&c.written).map(|x| format!("{}{value} {x}", c.cmd)), || c.cmd.clone());
        }
    }

    /// `set_false_path`, `set_multicycle_path`, then `set_max_delay` / `set_min_delay`, each in
    /// file order. An exception a timer refuses (`-through`, a pin pattern) is still written: the
    /// refusal is the timer's, and the file says what it said.
    fn write_exceptions(&self, o: &mut Out) {
        let sides = |args: &[(String, Objects)]| -> Option<String> {
            let mut t = String::new();
            for (k, v) in args {
                t += &format!(" {k} {}", objects(v)?);
            }
            Some(t)
        };
        for e in &self.exceptions {
            let (cmd, value) = match e.kind {
                ExcKind::FalsePath => ("set_false_path", String::new()),
                ExcKind::Multicycle(n) => ("set_multicycle_path", format!(" {n}")),
            };
            let flags: String = e.flags.iter().map(|f| format!(" {f}")).collect();
            o.line_or(sides(&e.args).map(|s| format!("{cmd}{value}{flags}{s}")), || cmd.to_string());
        }
        for p in &self.path_delays {
            let cmd = if p.max { "set_max_delay" } else { "set_min_delay" };
            if p.value.is_nan() {
                o.skip(format!("{cmd}: a value that is not a number"));
                continue;
            }
            let flags: String = p.flags.iter().map(|f| format!(" {f}")).collect();
            o.line_or(sides(&p.objects).map(|s| format!("{cmd}{flags}{s} {}", num(p.value))), || cmd.to_string());
        }
    }

    /// One environment command as written: `<cmd> <flags> [-clock c] <value> <objects>`.
    fn write_env_cmd(&self, o: &mut Out, e: &crate::sdc::EnvCmd) {
        let mut t = e.cmd.clone();
        for f in &e.flags {
            t += &format!(" {f}");
        }
        if let Some(c) = &e.clock {
            t += &format!(" -clock {}", if c.starts_with('[') { c.clone() } else { named("get_clocks", std::slice::from_ref(c)).unwrap_or_else(|| c.clone()) });
        }
        t += &format!(" {}", num(e.value));
        match &e.written {
            Some(w) => o.line_or(objects(w).map(|x| format!("{t} {x}")), || e.cmd.clone()),
            None => o.line(t),
        }
    }

    /// Loads, input slews, drives and derates.
    fn write_environment(&self, o: &mut Out) {
        for e in &self.env {
            self.write_env_cmd(o, e);
        }
        for d in &self.driving_cells {
            let mut t = String::from("set_driving_cell");
            for (k, v) in [("-lib_cell", &d.lib_cell), ("-library", &d.library), ("-pin", &d.pin), ("-from_pin", &d.from_pin)] {
                if let Some(v) = v {
                    t += &format!(" {k} {}", brace(std::slice::from_ref(v)).unwrap_or_else(|| v.clone()));
                }
            }
            // the reader keeps 0 for an absent input transition, which is the reference's default
            t += &format!(" -input_transition_rise {} -input_transition_fall {}", num(d.input_transition[0]), num(d.input_transition[1]));
            if let Some(m) = d.multiply_by {
                t += &format!(" -multiply_by {}", num(m));
            }
            for f in &d.flags {
                t += &format!(" {f}");
            }
            let objs: Option<Vec<String>> = d.written.iter().map(objects).collect();
            o.line_or(objs.map(|x| format!("{t} {}", x.join(" "))), || "set_driving_cell".into());
        }
        for e in &self.derates {
            self.write_env_cmd(o, e);
        }
    }

    /// The design's and the objects' transition, fanout and capacitance limits.
    fn write_design_rules(&self, o: &mut Out) {
        for (cmd, v, on_objects) in [
            ("set_max_transition", self.max_transition, self.max_transition_on_objects),
            ("set_max_fanout", self.max_fanout, self.max_fanout_on_objects),
        ] {
            if let Some(v) = v {
                o.line(format!("{cmd} {} [current_design]", num(v)));
            }
            if on_objects {
                o.skip(format!("{cmd} on objects other than the design (kept as a flag only)"));
            }
        }
        if let Some(v) = self.max_capacitance {
            o.line(format!("set_max_capacitance {} [current_design]", num(v)));
        }
        for (v, objs) in &self.max_capacitance_objects {
            if v.is_nan() {
                o.skip("set_max_capacitance: a value that is not a number".into());
                continue;
            }
            o.line_or(objects(objs).map(|x| format!("set_max_capacitance {} {x}", num(*v))), || "set_max_capacitance".into());
        }
    }

    /// Every command the reader did not keep, rejected, or could not evaluate.
    fn write_unwritten(&self, o: &mut Out) {
        let mut any = false;
        let mut head = |o: &mut Out| {
            if !any {
                o.section("Not written");
                any = true;
            }
        };
        for r in &self.rejected {
            head(o);
            o.skip(format!("rejected: {r}"));
        }
        for r in self.ignored.iter().filter(|r| !MODELLED_BUT_LISTED.contains(&r.as_str())) {
            head(o);
            o.skip(format!("not modelled: {r}"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(text: &str) -> (Written, Written) {
        let a = Sdc::parse(text).unwrap();
        let w1 = a.write(&WriteOpts::default());
        let b = Sdc::parse(&w1.text).unwrap();
        (w1, b.write(&WriteOpts::default()))
    }

    /// Writing is a FIXED POINT: what the writer wrote reads back to a model that writes the same
    /// text. Every kind the writer emits is in this file at least once.
    #[test]
    fn the_writer_is_a_fixed_point_of_read_then_write() {
        let text = "\
current_design top
set_units -time ns -capacitance pF
create_clock -name clk -period 10 -waveform {0 4} [get_ports clk]
create_clock -name vclk -period 5
create_clock -name clk2 -add -period 20 [get_ports clk]
create_generated_clock -name g -source [get_ports clk] -master_clock clk -divide_by 2 -invert [get_pins r/Q]
create_generated_clock -name e -source [get_ports clk] -edges {1 3 5} -edge_shift {0 1 0} [get_pins u/Z]
set_propagated_clock [all_clocks]
set_clock_transition -rise 0.1 [get_clocks clk]
set_clock_latency -source -early 0.2 [get_clocks clk]
set_clock_uncertainty -setup 0.05 [get_clocks clk]
set_clock_uncertainty -rise_from [get_clocks clk] -fall_to [get_clocks vclk] 0.1
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks vclk]
set_input_delay 1.5 -clock clk [all_inputs -no_clocks]
set_input_delay 2 -clock vclk -add_delay -max [get_ports {a[0] b}]
set_output_delay -0.25 -clock clk [all_outputs]
set_case_analysis 0 [get_pins -hierarchical u*/TE]
set_false_path -hold -from [get_clocks clk] -through [get_pins m/A] -to [get_ports y]
set_multicycle_path 2 -setup -end -from [list [get_ports a] [get_pins r/CK]] -to [all_registers -clock clk -data_pins]
set_max_delay -ignore_clock_latency -from [get_ports a] -to [get_ports y] 3.5
set_load -pin_load 0.01 [all_outputs]
set_input_transition 0.08 [all_inputs]
set_driving_cell -lib_cell BUF_X1 -pin Z -multiply_by 2 [get_ports a]
set_timing_derate -late -cell_delay 1.05
set_max_transition 1.5 [current_design]
set_max_fanout 16 [current_design]
set_max_capacitance 0.2 [get_ports y]
set_dont_touch [get_cells u1]
";
        let (w1, w2) = round_trip(text);
        // the second read sees the first's `# not written` comments as comments
        let written = w1.text.split("###############################################################################\n# Not written").next().unwrap();
        assert_eq!(written, w2.text, "read(write(x)) writes the same text");
        // what is not modelled is said, once, and not written
        assert_eq!(w1.not_written, vec!["not modelled: set_dont_touch".to_string()]);
        for want in [
            "current_design top",
            "create_clock -name {clk2} -add -period 20 [get_ports {clk}]",
            "create_generated_clock -name {g} -source [get_ports {clk}] -master_clock [get_clocks {clk}] -divide_by 2 -invert [get_pins {r/Q}]",
            "set_clock_uncertainty -rise_from [get_clocks {clk}] -fall_to [get_clocks {vclk}] 0.1",
            "set_input_delay 1.5 -clock [get_clocks {clk}] [all_inputs -no_clocks]",
            "set_case_analysis 0 [get_pins -hierarchical {u*/TE}]",
            "set_false_path -hold -from [get_clocks {clk}] -through [get_pins {m/A}] -to [get_ports {y}]",
            "set_multicycle_path 2 -setup -end -from [list [get_ports {a}] [get_pins {r/CK}]] -to [all_registers -data_pins -clock {clk}]",
        ] {
            assert!(w1.text.lines().any(|l| l == want), "missing `{want}` in\n{}", w1.text);
        }
    }

    /// A value that is not a number is not written, and is said not to be: writing `0` for an
    /// unresolved `$var` would ship a constraint the file never stated.
    #[test]
    fn an_unread_value_is_reported_not_written_as_zero() {
        let w = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\nset_input_delay $undefined -clock clk [get_ports a]\n")
            .unwrap()
            .write(&WriteOpts::default());
        assert!(!w.text.lines().any(|l| l.starts_with("set_input_delay")), "{}", w.text);
        assert_eq!(w.not_written.len(), 1, "{:?}", w.not_written);
    }

    /// Values are written in the shortest form that reads back to the same f64.
    #[test]
    fn values_round_trip_exactly() {
        for v in [0.1, 1.0 / 3.0, 1e-12, -0.0, 123456.789] {
            assert_eq!(num(v).parse::<f64>().unwrap(), v);
        }
        assert_eq!(num(-0.0), "0");
    }
}
