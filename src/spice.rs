//! Minimal SPICE (transistor-level) netlist reader — a device-level netlist front
//! end alongside the gate-level [`crate::netlist`]/[`crate::yosys_json`] readers. The
//! primary consumer is LVS (schematic-vs-extracted compare), but it is shared foundation.
//!
//! Parses a flat or hierarchical netlist into typed devices (kind, instance
//! name, ordered terminal nets, model/value). Handles `+` line continuation,
//! `*`/`;`/`$` comments, `.subckt`/`.ends` scoping, `.global`, and the common
//! device kinds (M, Q, R, C, L, D, X). Other dot-commands are ignored.
//! Case-insensitive.
//!
//! **Hierarchical flattening**: every `X` subckt instance whose definition is
//! present is recursively expanded down to primitive devices — formal ports are
//! bound to the actual nets at the call site, internal nets are uniquified by the
//! instance path, and `.global` (plus SPICE node `0`) names stay global. This is
//! what turns a cell-level connectivity check into a *transistor-level* compare:
//! two layouts that instantiate the same cells but wire their internals
//! differently now diverge. An `X` whose subckt is *not* defined degrades to an
//! opaque device (its connectivity is still mapped through the hierarchy).
//!
//! **PDK device recognition**: an `X` instance whose model is a device (a
//! transistor/diode/BJT model, e.g. `sky130_fd_pr__nfet_01v8`) is treated as that
//! primitive device kind, *not* descended into — so an extracted layout (magic
//! writes every transistor as `X.. d g s b <model> w=.. l=..`) compares identically
//! to a schematic that writes the same transistor as `M`. See [`recognize_device_kind`].
//!
//! Device **parameters** (MOSFET `w`/`l`/`nf`/`m`, and the value of an `R`/`C`/`L`)
//! are captured too, so the comparator can check them — a layout that matches
//! topologically but draws a transistor at the wrong width is a real LVS error.
//!
//! Depth reserved: inline `X` parameters and `.param` evaluation.

use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq)]
pub struct Device {
    pub kind: char,         // normalized uppercase first letter (M/Q/R/C/L/D/X)
    pub name: String,       // instance name (as written)
    pub nodes: Vec<String>, // ordered terminal nets
    pub model: String,      // model / subckt / value (may be empty)
    /// Numeric device parameters in SI units (e.g. `w`, `l`, `nf`, `m`, or the
    /// passive `value`). Keys are lowercased; SPICE engineering suffixes resolved.
    pub params: BTreeMap<String, f64>,
}

/// Parse a SPICE numeric literal with an optional engineering suffix
/// (`1k`, `2.5u`, `0.18µ`, `3meg`, `1e-9`), returning the value in base SI units.
/// Trailing unit letters after the multiplier (e.g. the `f` in `1.2pf`) are
/// ignored. Returns `None` if there is no leading number.
pub fn parse_spice_num(tok: &str) -> Option<f64> {
    let t = tok.trim();
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    while i < b.len() && (b[i].is_ascii_digit() || b[i] == b'.') {
        i += 1;
    }
    // optional exponent `e[+-]?digits` (back off if it isn't a real exponent)
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let save = i;
        i += 1;
        if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
            i += 1;
        }
        let mut got = false;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            got = true;
        }
        if !got {
            i = save;
        }
    }
    let base: f64 = t[..i].parse().ok()?;
    let suffix = t[i..].trim().to_ascii_lowercase();
    let mult = if suffix.starts_with("meg") {
        1e6
    } else {
        match suffix.chars().next() {
            None => 1.0,
            Some('t') => 1e12,
            Some('g') => 1e9,
            Some('k') => 1e3,
            Some('m') => 1e-3, // 'meg' handled above; bare 'm' = milli
            Some('u') | Some('µ') => 1e-6,
            Some('n') => 1e-9,
            Some('p') => 1e-12,
            Some('f') => 1e-15,
            Some('a') => 1e-18,
            _ => 1.0, // unrecognized trailing unit -> treat as a bare number
        }
    };
    Some(base * mult)
}

#[derive(Debug, Clone, Default)]
pub struct Netlist {
    pub name: String,
    pub ports: Vec<String>,
    pub devices: Vec<Device>,
}

#[derive(Debug)]
pub struct SpiceError(pub String);
impl std::fmt::Display for SpiceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "spice error: {}", self.0)
    }
}
impl std::error::Error for SpiceError {}

/// Terminal count for a device kind (X is variable -> handled separately).
fn fixed_terms(kind: char) -> Option<usize> {
    match kind {
        'M' => Some(4), // d g s b
        'Q' => Some(3), // c b e
        'R' | 'C' | 'L' | 'D' => Some(2),
        _ => None,
    }
}

impl Netlist {
    pub fn load(path: &str) -> Result<Netlist, SpiceError> {
        let text = std::fs::read_to_string(path).map_err(|e| SpiceError(format!("{path}: {e}")))?;
        Netlist::parse(&text, None)
    }

    /// Parse `text`; if `top` is given, return that `.subckt`, else the named
    /// single subckt, else the flat top-level devices. The chosen cell is
    /// **flattened to primitive devices** against the full subckt table (see
    /// [`flatten`]).
    pub fn parse(text: &str, top: Option<&str>) -> Result<Netlist, SpiceError> {
        let stmts = statements(text);
        // collect subckts + global nets
        let mut subckts: Vec<Netlist> = Vec::new();
        let mut flat: Vec<Device> = Vec::new();
        let mut globals: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        globals.insert("0".into()); // SPICE global ground
        let mut cur: Option<Netlist> = None;

        for st in &stmts {
            let toks: Vec<&str> = st.split_whitespace().collect();
            if toks.is_empty() {
                continue;
            }
            let head = toks[0].to_ascii_lowercase();
            if head == ".subckt" {
                let name = toks.get(1).map(|s| s.to_string()).unwrap_or_default();
                let ports = toks[2..].iter().map(|s| s.to_string()).collect();
                cur = Some(Netlist { name, ports, devices: Vec::new() });
                continue;
            }
            if head == ".ends" {
                if let Some(n) = cur.take() {
                    subckts.push(n);
                }
                continue;
            }
            if head == ".global" {
                globals.extend(toks[1..].iter().map(|s| s.to_string()));
                continue;
            }
            if head.starts_with('.') {
                continue; // .model/.param/.include/.end/...
            }
            if let Some(dev) = parse_device(&toks)? {
                match cur.as_mut() {
                    Some(n) => n.devices.push(dev),
                    None => flat.push(dev),
                }
            }
        }

        // subckt-definition table (case-insensitive lookup by name)
        let table: FlatMap<String, Netlist> =
            subckts.iter().map(|s| (s.name.to_ascii_lowercase(), s.clone())).collect();

        let chosen = if let Some(want) = top {
            table
                .get(&want.to_ascii_lowercase())
                .cloned()
                .ok_or_else(|| SpiceError(format!("subckt {want:?} not found")))?
        } else {
            match subckts.len() {
                0 => Netlist { name: "(top)".into(), ports: Vec::new(), devices: flat },
                1 => subckts.into_iter().next().unwrap(),
                _ => {
                    return Err(SpiceError(format!(
                        "{} subckts; pass `top:` to choose ({})",
                        subckts.len(),
                        subckts.iter().map(|s| s.name.as_str()).collect::<Vec<_>>().join(", ")
                    )))
                }
            }
        };

        Ok(flatten(&chosen, &table, &globals))
    }
}

use std::collections::BTreeMap as FlatMap;

/// Resolves a node name in the current expansion frame (port subst + path prefix).
type Resolve<'a> = dyn Fn(&str, &FlatMap<String, String>, &str) -> String + 'a;

/// Recursively expand the cell's `X` instances down to primitive devices.
///
/// At each level a node is resolved in the current namespace: a formal port maps
/// to the actual net passed at the call site, a global stays itself, and any
/// other (internal) net is prefixed with the instance path so two instances of
/// the same cell get distinct internal nets. An `X` whose subckt is absent from
/// `table` is emitted as an opaque device with its nodes mapped — connectivity is
/// preserved even when the definition is unavailable.
pub fn flatten(
    top: &Netlist,
    table: &FlatMap<String, Netlist>,
    globals: &std::collections::BTreeSet<String>,
) -> Netlist {
    let mut out: Vec<Device> = Vec::new();
    // current frame: formal->resolved-actual substitution + instance-path prefix
    let resolve = |node: &str, subst: &FlatMap<String, String>, prefix: &str| -> String {
        if let Some(actual) = subst.get(node) {
            actual.clone()
        } else if globals.contains(node) {
            node.to_string()
        } else {
            format!("{prefix}{node}")
        }
    };
    // explicit recursion guarded by depth; globals/ports captured in `resolve`
    expand(top, table, &FlatMap::new(), "", 0, &resolve, &mut out);
    Netlist { name: top.name.clone(), ports: top.ports.clone(), devices: out }
}

#[allow(clippy::too_many_arguments)]
fn expand(
    cell: &Netlist,
    table: &FlatMap<String, Netlist>,
    subst: &FlatMap<String, String>,
    prefix: &str,
    depth: usize,
    resolve: &Resolve<'_>,
    out: &mut Vec<Device>,
) {
    for d in &cell.devices {
        let nodes: Vec<String> = d.nodes.iter().map(|n| resolve(n, subst, prefix)).collect();
        // A subckt instance whose model is a PDK **device** (a transistor/diode/BJT,
        // not a hierarchical cell) is a primitive, not a hierarchy to descend into:
        // emit it as that device kind so it compares identically to the schematic's
        // `M`/`D`/`Q` form. This also stops us from flattening a device subckt's
        // parasitic internals (the `d g s b` MOSFET vs its diode-laden model body).
        if d.kind == 'X' {
            if let Some(k) = recognize_device_kind(&d.model, nodes.len()) {
                out.push(Device {
                    kind: k,
                    name: format!("{prefix}{}", d.name),
                    nodes,
                    model: d.model.clone(),
                    params: d.params.clone(),
                });
                continue;
            }
        }
        let sub = (d.kind == 'X')
            .then(|| table.get(&d.model.to_ascii_lowercase()))
            .flatten();
        match sub {
            Some(def) if depth < MAX_HIER_DEPTH && def.ports.len() == nodes.len() => {
                // bind formals -> resolved actuals; recurse into the child namespace
                let child_subst: FlatMap<String, String> =
                    def.ports.iter().cloned().zip(nodes.iter().cloned()).collect();
                let child_prefix = format!("{prefix}{}/", d.name);
                expand(def, table, &child_subst, &child_prefix, depth + 1, resolve, out);
            }
            _ => out.push(Device {
                kind: d.kind,
                name: format!("{prefix}{}", d.name),
                nodes,
                model: d.model.clone(),
                params: d.params.clone(),
            }),
        }
    }
}

/// Recursion guard — a malformed self-referential subckt won't expand forever.
const MAX_HIER_DEPTH: usize = 100;

/// Recognize a subckt instance whose model is a PDK **device** — a transistor,
/// diode, or BJT — rather than a hierarchical cell, so it compares as the primitive
/// device a schematic writes directly (`M`/`D`/`Q`). Most PDKs wrap each device in a
/// model subckt: magic's `ext2spice` emits every transistor as
/// `X.. d g s b sky130_fd_pr__nfet_01v8 w=.. l=..`, while a gate-level netlist writes
/// the same transistor as `M`. Without this, every transistor in an extracted layout
/// is an opaque `X` that can never match its `M` counterpart — the whole device
/// population diverges. Matched by the near-universal device-model substrings, gated
/// by terminal count so a hierarchical cell (more ports, `_sc_`/`_lib_` names) is
/// never mistaken for a device. Returns the primitive device kind.
fn recognize_device_kind(model: &str, n_nodes: usize) -> Option<char> {
    let m = model.to_ascii_lowercase();
    let has = |p: &str| m.contains(p);
    match n_nodes {
        // MOSFET: drain gate source bulk
        4 if has("nfet") || has("pfet") || has("mosfet") || has("nmos") || has("pmos") => Some('M'),
        // BJT: collector base emitter
        3 if has("_npn") || has("_pnp") || has("bjt") => Some('Q'),
        // diode: anode cathode
        2 if has("diode") => Some('D'),
        _ => None,
    }
}

fn parse_device(toks: &[&str]) -> Result<Option<Device>, SpiceError> {
    let name = toks[0].to_string();
    let kind = name.chars().next().unwrap_or('?').to_ascii_uppercase();
    if kind == 'X' {
        // Xname n1 .. nk subcktname [p=v ...]
        // Split off trailing `key=value` params (device geometry: w/l/ad/pd/as/ps/…);
        // of the remaining bare tokens the **last** is the subckt/model name and the
        // rest are the terminal nets. Magic's `ext2spice` writes each transistor this
        // way (`X.. d g s b sky130_fd_pr__nfet_01v8 w=.. l=..`), so the params must not
        // be mistaken for the model, nor the model/params for nets.
        let mut params: BTreeMap<String, f64> = BTreeMap::new();
        let mut bare: Vec<&str> = Vec::new();
        for tok in &toks[1..] {
            match tok.split_once('=') {
                Some((k, v)) => {
                    if let Some(n) = parse_spice_num(v) {
                        params.insert(k.to_ascii_lowercase(), n);
                    }
                }
                None => bare.push(tok),
            }
        }
        if bare.len() < 2 {
            return Ok(None); // need at least one terminal + a subckt name
        }
        let model = bare[bare.len() - 1].to_string();
        let nodes = bare[..bare.len() - 1].iter().map(|s| s.to_string()).collect();
        return Ok(Some(Device { kind, name, nodes, model, params }));
    }
    let Some(nt) = fixed_terms(kind) else {
        return Ok(None); // unknown device kind -> skip (don't guess connectivity)
    };
    if toks.len() < 1 + nt {
        return Err(SpiceError(format!("device {name} needs {nt} terminals: {:?}", toks.join(" "))));
    }
    let nodes = toks[1..1 + nt].iter().map(|s| s.to_string()).collect();
    let rest = &toks[1 + nt..];
    let (model, params) = device_params(kind, rest);
    Ok(Some(Device { kind, name, nodes, model, params }))
}

/// From the tokens after a device's terminals, split out the model name and the
/// numeric parameters. `key=value` tokens become params (lowercased key); a bare
/// leading number is the passive **value** (`R1 a b 1k` → `value = 1000`); a bare
/// non-numeric token is the model/subckt name (`Mp … pfet`).
fn device_params(kind: char, rest: &[&str]) -> (String, BTreeMap<String, f64>) {
    let mut model = String::new();
    let mut params: BTreeMap<String, f64> = BTreeMap::new();
    for (i, tok) in rest.iter().enumerate() {
        if let Some((k, v)) = tok.split_once('=') {
            if let Some(n) = parse_spice_num(v) {
                params.insert(k.to_ascii_lowercase(), n);
            }
        } else if i == 0 {
            // first bare token: a passive's value if numeric, else the model name
            match (matches!(kind, 'R' | 'C' | 'L'), parse_spice_num(tok)) {
                (true, Some(n)) => {
                    params.insert("value".into(), n);
                }
                _ => model = tok.to_string(),
            }
        } else if model.is_empty() && parse_spice_num(tok).is_none() {
            model = tok.to_string();
        }
    }
    (model, params)
}

/// Split into statements: strip comments, join `+` continuations.
fn statements(text: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for raw in text.lines() {
        // strip `*` full-line and `;`/`$` inline comments
        let mut line = raw;
        if line.trim_start().starts_with('*') {
            continue;
        }
        if let Some(i) = line.find([';', '$']) {
            line = &line[..i];
        }
        let line = line.trim_end();
        if line.trim().is_empty() {
            continue;
        }
        if line.trim_start().starts_with('+') {
            // continuation -> append to previous statement
            let cont = line.trim_start().trim_start_matches('+').trim();
            if let Some(last) = out.last_mut() {
                last.push(' ');
                last.push_str(cont);
                continue;
            }
        }
        out.push(line.trim().to_string());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const NL: &str = "\
.subckt inv A Y VDD VSS
Mp Y A VDD VDD pfet w=1 l=0.15
Mn Y A VSS VSS nfet w=0.5 l=0.15
.ends
";

    #[test]
    fn parses_subckt_devices() {
        let n = Netlist::parse(NL, None).unwrap();
        assert_eq!(n.name, "inv");
        assert_eq!(n.ports, ["A", "Y", "VDD", "VSS"]);
        assert_eq!(n.devices.len(), 2);
        assert_eq!(n.devices[0].kind, 'M');
        assert_eq!(n.devices[0].nodes, ["Y", "A", "VDD", "VDD"]);
        assert_eq!(n.devices[0].model, "pfet");
    }

    #[test]
    fn continuation_and_comments() {
        let t = "* header\nR1 a b\n+ 1k ; trailing\nC1 b 0 1p\n";
        let n = Netlist::parse(t, None).unwrap();
        assert_eq!(n.devices.len(), 2);
        assert_eq!(n.devices[0].nodes, ["a", "b"]);
        // a passive's value is captured as a numeric param, not the model string
        assert_eq!(n.devices[0].params.get("value"), Some(&1000.0));
        assert_eq!(n.devices[1].params.get("value"), Some(&1e-12));
    }

    fn approx(a: Option<f64>, b: f64) -> bool {
        a.map(|v| (v - b).abs() <= b.abs() * 1e-12).unwrap_or(false)
    }

    #[test]
    fn parses_mosfet_w_l_and_suffixes() {
        let n = Netlist::parse("M1 d g s b nfet w=2u l=0.15u nf=4\n", None).unwrap();
        let p = &n.devices[0].params;
        assert_eq!(n.devices[0].model, "nfet");
        assert!(approx(p.get("w").copied(), 2e-6));
        assert!(approx(p.get("l").copied(), 0.15e-6));
        assert!(approx(p.get("nf").copied(), 4.0));
    }

    #[test]
    fn spice_num_suffixes() {
        assert!(approx(parse_spice_num("1k"), 1e3));
        assert!(approx(parse_spice_num("3meg"), 3e6));
        assert!(approx(parse_spice_num("2.5u"), 2.5e-6));
        assert!(approx(parse_spice_num("1p"), 1e-12));
        assert!(approx(parse_spice_num("1.2pf"), 1.2e-12)); // trailing unit ignored
        assert!(approx(parse_spice_num("1e-9"), 1e-9));
        assert_eq!(parse_spice_num("nfet"), None);
    }

    const HIER: &str = "\
.subckt inv A Y VDD VSS
Mp Y A VDD VDD pfet
Mn Y A VSS VSS nfet
.ends
.subckt buf A Y VDD VSS
Xi1 A M VDD VSS inv
Xi2 M Y VDD VSS inv
.ends
";

    #[test]
    fn flattens_to_transistors() {
        let n = Netlist::parse(HIER, Some("buf")).unwrap();
        // two inverters expand to four MOSFETs; no X survives
        assert_eq!(n.devices.len(), 4);
        assert!(n.devices.iter().all(|d| d.kind == 'M'));
        // ports stay at the boundary; the internal node M is shared between the
        // two instances (passed as an actual), not path-prefixed
        let nets: std::collections::BTreeSet<&str> =
            n.devices.iter().flat_map(|d| d.nodes.iter().map(|s| s.as_str())).collect();
        assert!(nets.contains("M"), "shared internal net M should survive: {nets:?}");
        assert!(nets.contains("VDD") && nets.contains("VSS"));
        // the inverter's own internal nodes (none here) would be path-prefixed;
        // instance device names are path-qualified and unique
        let names: std::collections::BTreeSet<&str> =
            n.devices.iter().map(|d| d.name.as_str()).collect();
        assert_eq!(names.len(), 4, "instance device names must be unique: {names:?}");
    }

    #[test]
    fn x_device_with_trailing_params_parses_model_and_nets() {
        // magic ext2spice form: X<name> d g s b <model> <geom params>. The model is
        // the last *bare* token; the params must not be folded into the node list.
        let t = "\
.subckt c d g s VPWR VGND VPB VNB
X0 d g s VNB sky130_fd_pr__nfet_01v8 ad=0.044 pd=0.63 w=0.42 l=0.15
.ends
";
        let n = Netlist::parse(t, Some("c")).unwrap();
        assert_eq!(n.devices.len(), 1);
        let dev = &n.devices[0];
        assert_eq!(dev.kind, 'M', "an nfet device model must be recognized as a MOSFET");
        assert_eq!(dev.model, "sky130_fd_pr__nfet_01v8");
        assert_eq!(dev.nodes, ["d", "g", "s", "VNB"], "4 terminals, no model/params leaking in");
        assert!((dev.params.get("w").copied().unwrap() - 0.42).abs() < 1e-12);
        assert!((dev.params.get("l").copied().unwrap() - 0.15).abs() < 1e-12);
    }

    #[test]
    fn hierarchical_cell_is_not_mistaken_for_a_device() {
        // a std-cell subckt call (defined, many ports, no fet/mos in its name) must
        // still flatten normally, not be swallowed as a "device".
        let t = "\
.subckt inv A Y VPWR VGND VPB VNB
X0 Y A VPWR VPB sky130_fd_pr__pfet_01v8 w=1 l=0.15
X1 Y A VGND VNB sky130_fd_pr__nfet_01v8 w=0.65 l=0.15
.ends
.subckt top A Y VPWR VGND VPB VNB
Xu0 A Y VPWR VGND VPB VNB inv
.ends
";
        let n = Netlist::parse(t, Some("top")).unwrap();
        assert_eq!(n.devices.len(), 2, "the cell expands to its two transistors");
        assert!(n.devices.iter().all(|d| d.kind == 'M'));
    }

    #[test]
    fn undefined_subckt_stays_opaque() {
        let t = "\
.subckt top A Y VDD VSS
Xb A Y VDD VSS blackbox
.ends
";
        let n = Netlist::parse(t, Some("top")).unwrap();
        assert_eq!(n.devices.len(), 1);
        assert_eq!(n.devices[0].kind, 'X');
        assert_eq!(n.devices[0].nodes, ["A", "Y", "VDD", "VSS"]);
    }
}
