//! `current_instance`: names read relative to an instance, as the reference reads them.
//!
//! In the reference, `current_instance u1` sets the instance its pin, cell and net queries match
//! under (`find_pins_matching`, `find_instances_matching`, `find_nets_matching` all start from
//! `Sta::currentInstance`), while `get_ports` always matches the TOP cell's ports and a clock is
//! no object of the netlist. Its argument is an ABSOLUTE instance path (`get_instance_error`,
//! `find_instance`), and no argument goes back to the top.
//!
//! Here a pattern under a scope is written with the scope in front (`[get_pins a/Z]` under `u1`
//! is `[get_pins u1/a/Z]`): the reader's names are full paths, and the writer writes them so.
//!
//! ⛔ **What only the design can decide is refused, not guessed.** A BARE name under a scope is
//! found by kind (`get_object_args`: a clock, a library cell, a cell under the scope, a top port,
//! a pin, a net under the scope — the first kind that has it), and only the design knows which
//! kind a name is. A clock this file defined is a clock; any other bare object name refuses the
//! command. So does `-hierarchical` under a scope, whose match starts at every level below it.

use super::{brace_word, tokenize};

/// Keys whose value names objects (so a bare value is ambiguous under a scope). Every other key
/// takes a value (`-name`, `-period`, `-type`, `-library`, …) or clocks (`-clock`, `-clocks`,
/// `-master_clock`), which a scope does not change.
const OBJECT_KEYS: [&str; 13] = [
    "-from", "-to", "-rise_from", "-fall_from", "-rise_to", "-fall_to", "-through", "-rise_through", "-fall_through",
    "-object_list", "-reference_pin", "-source", "-of_objects",
];

/// Keys that take objects in the commands that have them, but a VALUE in these commands (a
/// library port name, a group of clocks).
fn key_is_value(cmd: &str, key: &str) -> bool {
    matches!((cmd, key), ("set_disable_timing" | "unset_disable_timing", "-from" | "-to"))
}

/// Commands whose first positional is a value, not objects.
fn first_positional_is_value(cmd: &str) -> bool {
    matches!(cmd, "set_case_analysis" | "set_wire_load_mode" | "set_operating_conditions" | "set_wire_load_selection_group")
}

/// Commands with no objects at all, or whose words this check must not judge.
fn unscoped(cmd: &str) -> bool {
    matches!(
        cmd,
        "set" | "puts" | "echo" | "current_instance" | "set_current_instance" | "current_design" | "set_units" | "set_unit"
            | "set_hierarchy_separator" | "set_wire_load_mode" | "set_max_area" | "set_max_dynamic_power"
            | "set_max_leakage_power" | "set_operating_conditions" | "set_wire_load_min_block_size"
            | "set_level_shifter_strategy" | "set_level_shifter_threshold"
    )
}

/// `line` with its queries read under `scope` (see the module documentation), or why it is
/// refused. `clocks` are the clocks defined so far: a bare name that is one is a clock.
pub(super) fn scoped_line(line: &str, scope: &str, clocks: &[String]) -> Result<String, String> {
    let toks = tokenize(line);
    let Some(cmd) = toks.first().cloned() else { return Ok(line.to_string()) };
    if unscoped(&cmd) {
        return Ok(line.to_string());
    }
    let mut out = vec![cmd.clone()];
    let keys: &[&str] = crate::sdc_flags::FLAGS.iter().find(|(n, _, _)| *n == cmd).map_or(&[][..], |(_, k, _)| *k);
    let mut positional = 0usize;
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        let is_option = t.starts_with('-') && t.parse::<f64>().is_err() && t.len() > 1;
        if is_option {
            out.push(t.clone());
            let takes_value = keys.contains(&t.as_str()) || OBJECT_KEYS.contains(&t.as_str()) || t == "-group";
            if takes_value {
                if let Some(v) = toks.get(k + 1) {
                    let objects = (OBJECT_KEYS.contains(&t.as_str()) || t == "-group") && !key_is_value(&cmd, t);
                    out.push(if objects { scoped_objects(v, scope, clocks)? } else { word(v, scope)? });
                    k += 2;
                    continue;
                }
            }
            k += 1;
            continue;
        }
        let value = t.parse::<f64>().is_ok() || (positional == 0 && first_positional_is_value(&cmd));
        out.push(if value { word(t, scope)? } else { scoped_objects(t, scope, clocks)? });
        positional += 1;
        k += 1;
    }
    Ok(out.join(" "))
}

/// A value word: kept, with any query in it read under the scope.
fn word(w: &str, scope: &str) -> Result<String, String> {
    if w.starts_with('[') {
        return query(w, scope, &[]);
    }
    Ok(brace_word(w))
}

/// An objects word: a query read under the scope, or a bare name list — each name a clock, or
/// refused.
fn scoped_objects(w: &str, scope: &str, clocks: &[String]) -> Result<String, String> {
    if w.starts_with('[') {
        return query(w, scope, clocks);
    }
    for n in w.split_whitespace() {
        if !clocks.iter().any(|c| c == n) {
            return Err(format!(
                "the bare name `{n}` under current_instance {scope}: the reference finds it by kind (clock, cell, port, pin or net), which only the design can say"
            ));
        }
    }
    Ok(brace_word(w))
}

/// One `[...]` query under the scope: a pin, cell or net query's patterns get the scope in front;
/// a `list` / `concat` is read element by element; any other query is kept as written (ports and
/// clocks are not scoped), its own nested queries read under the scope.
fn query(w: &str, scope: &str, clocks: &[String]) -> Result<String, String> {
    let inner = w.strip_prefix('[').and_then(|i| i.strip_suffix(']')).unwrap_or(w);
    let toks = tokenize(inner.trim());
    let Some(q) = toks.first().cloned() else { return Ok(w.to_string()) };
    let scoped = matches!(q.as_str(), "get_pins" | "get_cells" | "get_nets");
    if scoped && toks.iter().any(|t| t == "-hierarchical" || t == "-hier") {
        return Err(format!("`{q} -hierarchical` under current_instance {scope}: a match from every level below it is not modelled"));
    }
    let keys: &[&str] = crate::sdc_flags::FLAGS.iter().find(|(n, _, _)| *n == q).map_or(&[][..], |(_, k, _)| *k);
    let mut out = vec![q.clone()];
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        if t.starts_with('-') && t.len() > 1 && t.parse::<f64>().is_err() {
            out.push(t.clone());
            if keys.contains(&t.as_str()) {
                if let Some(v) = toks.get(k + 1) {
                    out.push(if v.starts_with('[') { query(v, scope, clocks)? } else { brace_word(v) });
                    k += 2;
                    continue;
                }
            }
            k += 1;
            continue;
        }
        let item = if t.starts_with('[') {
            query(t, scope, clocks)?
        } else if scoped {
            let names: Vec<String> = t.split_whitespace().map(|n| format!("{scope}/{n}")).collect();
            brace_word(&names.join(" "))
        } else if q == "list" || q == "concat" {
            scoped_objects(t, scope, clocks)?
        } else {
            brace_word(t)
        };
        out.push(item);
        k += 1;
    }
    Ok(format!("[{}]", out.join(" ")))
}

#[cfg(test)]
mod tests {
    use super::super::Sdc;

    /// Rule (`find_*_matching` from `Sta::currentInstance`; `get_ports` from the top cell): under
    /// `current_instance u1`, pin, cell and net patterns are under `u1`; ports and clocks are not;
    /// no argument goes back to the top.
    #[test]
    fn names_under_current_instance_are_under_it() {
        let s = Sdc::parse(
            "create_clock -name clk -period 1 [get_ports clk]\ncurrent_instance u1\nset_false_path -from [get_pins r1/CK] -to [get_ports out]\nset_disable_timing -from A -to Z [get_cells {g1 g2}]\nset_false_path -from clk -through [get_nets n1]\ncurrent_instance\nset_false_path -to [get_pins r2/D]\n",
        )
        .expect("reads");
        assert!(s.rejected.is_empty(), "{:?}", s.rejected);
        assert_eq!(s.exceptions[0].args[0].1.names, ["u1/r1/CK"]);
        assert_eq!(s.exceptions[0].args[1].1.names, ["out"]);
        assert_eq!(s.disabled_timing[0].objects, ["u1/g1", "u1/g2"]);
        assert_eq!(s.disabled_timing[0].keys, [("-from".to_string(), "A".to_string()), ("-to".to_string(), "Z".to_string())]);
        assert_eq!(s.exceptions[1].args[1].1.names, ["u1/n1"]);
        assert_eq!(s.exceptions[2].args[0].1.names, ["r2/D"], "back at the top");
    }

    /// A bare name under a scope is found by kind, which needs the design: refused, not guessed;
    /// so is `-hierarchical` under a scope.
    #[test]
    fn what_needs_the_design_under_a_scope_is_refused() {
        let s = Sdc::parse("current_instance u1\nset_false_path -from r1/CK\nset_false_path -to [get_pins -hierarchical */D]\nset_case_analysis 0 [get_pins m/S]\n").expect("reads");
        assert_eq!(s.exceptions.len(), 0);
        assert_eq!(s.rejected.len(), 2, "{:?}", s.rejected);
        assert_eq!(s.case_values[0].objects, ["u1/m/S"]);
    }
}
