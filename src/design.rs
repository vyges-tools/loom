//! The in-memory design database — loom's reason to exist.
//!
//! **Parse once, query many.** Load the standard formats (structural Verilog
//! netlist, Liberty `.lib`, SDC, SPEF) into one [`Design`] that every engine
//! queries, instead of each tool re-parsing its own copy. This is the data plane
//! the engines weave on.
//!
//! It is also the home of **cross-step state** ([`Design::steps`]) — what was
//! loaded, in order, with a one-line summary — the runtime sibling of
//! `vyges-metadata.json`, and the seed of the provenance/caching `sley` will use.

use crate::{def, lef, liberty, netlist, sdc, spef};

/// One recorded load/step — the seed of loom's cross-step state (provenance).
#[derive(Debug, Clone)]
pub struct Step {
    /// `"netlist" | "liberty" | "sdc" | "spef"`.
    pub kind: &'static str,
    pub path: String,
    /// Human-readable one-liner (counts).
    pub summary: String,
}

/// The shared in-memory design. Engines read from this rather than re-parsing.
#[derive(Debug, Default)]
pub struct Design {
    pub netlist: Option<netlist::Netlist>,
    /// One entry per `.lib` loaded (corners / multiple libraries).
    pub libs: Vec<liberty::Lib>,
    pub sdc: Option<sdc::Sdc>,
    pub spef: Option<spef::Spef>,
    pub lef: Option<lef::Lef>,
    pub def: Option<def::Def>,
    /// Cross-step state: every load, in order (provenance).
    pub steps: Vec<Step>,
}

/// Error loading a file into the design (wraps each parser's error).
#[derive(Debug)]
pub struct DesignError(pub String);
impl std::fmt::Display for DesignError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for DesignError {}

impl Design {
    pub fn new() -> Self {
        Self::default()
    }

    /// Load a structural Verilog netlist (`.v`/`.sv`).
    pub fn load_netlist(&mut self, path: &str) -> Result<(), DesignError> {
        let nl = netlist::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!(
            "module {} — {} inputs, {} outputs, {} instances",
            nl.module,
            nl.inputs.len(),
            nl.outputs.len(),
            nl.insts.len()
        );
        self.steps.push(Step { kind: "netlist", path: path.into(), summary });
        self.netlist = Some(nl);
        Ok(())
    }

    /// Load a Liberty timing library (`.lib`). Appends — call once per corner.
    pub fn load_liberty(&mut self, path: &str) -> Result<(), DesignError> {
        let lib = liberty::Lib::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!("{} cells", lib.cells.len());
        self.steps.push(Step { kind: "liberty", path: path.into(), summary });
        self.libs.push(lib);
        Ok(())
    }

    /// Load SDC constraints (`.sdc`).
    pub fn load_sdc(&mut self, path: &str) -> Result<(), DesignError> {
        let s = sdc::Sdc::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!(
            "{} clocks, {} input-delays, {} output-delays, {} exceptions",
            s.clocks.len(),
            s.input_delays.len(),
            s.output_delays.len(),
            s.exceptions.len()
        );
        self.steps.push(Step { kind: "sdc", path: path.into(), summary });
        self.sdc = Some(s);
        Ok(())
    }

    /// Load SPEF parasitics (`.spef`).
    pub fn load_spef(&mut self, path: &str) -> Result<(), DesignError> {
        let sp = spef::Spef::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!("{} nets", sp.nets.len());
        self.steps.push(Step { kind: "spef", path: path.into(), summary });
        self.spef = Some(sp);
        Ok(())
    }

    /// Load a tech-LEF (`.lef`/`.tlef`).
    pub fn load_lef(&mut self, path: &str) -> Result<(), DesignError> {
        let lf = lef::Lef::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!("{} layers", lf.layers.len());
        self.steps.push(Step { kind: "lef", path: path.into(), summary });
        self.lef = Some(lf);
        Ok(())
    }

    /// Load a DEF (`.def`).
    pub fn load_def(&mut self, path: &str) -> Result<(), DesignError> {
        let d = def::Def::load(path).map_err(|e| DesignError(e.to_string()))?;
        let summary = format!(
            "{} signal nets, {} power nets, {} components",
            d.nets.len(),
            d.power_nets.len(),
            d.comps.len()
        );
        self.steps.push(Step { kind: "def", path: path.into(), summary });
        self.def = Some(d);
        Ok(())
    }

    /// Dispatch a file to the right reader by extension. Returns the `kind`
    /// loaded. Unknown extensions are an error (loom is explicit, no silent skip).
    pub fn load(&mut self, path: &str) -> Result<&'static str, DesignError> {
        let ext = std::path::Path::new(path)
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        match ext.as_str() {
            "v" | "sv" | "vg" => self.load_netlist(path).map(|_| "netlist"),
            "lib" => self.load_liberty(path).map(|_| "liberty"),
            "sdc" => self.load_sdc(path).map(|_| "sdc"),
            "spef" => self.load_spef(path).map(|_| "spef"),
            "lef" | "tlef" => self.load_lef(path).map(|_| "lef"),
            "def" => self.load_def(path).map(|_| "def"),
            other => Err(DesignError(format!(
                "{path}: unknown extension '.{other}' (expected .v/.sv, .lib, .sdc, .spef, .lef, .def)"
            ))),
        }
    }

    /// Total cells across all loaded Liberty libraries.
    pub fn lib_cell_count(&self) -> usize {
        self.libs.iter().map(|l| l.cells.len()).sum()
    }

    /// Human-readable multi-line summary of what loom is holding.
    pub fn summary(&self) -> String {
        let mut out = String::new();
        if let Some(nl) = &self.netlist {
            out.push_str(&format!(
                "netlist : module {} — {} inputs, {} outputs, {} instances\n",
                nl.module,
                nl.inputs.len(),
                nl.outputs.len(),
                nl.insts.len()
            ));
        }
        if !self.libs.is_empty() {
            out.push_str(&format!(
                "liberty : {} librar{} — {} cells total\n",
                self.libs.len(),
                if self.libs.len() == 1 { "y" } else { "ies" },
                self.lib_cell_count()
            ));
        }
        if let Some(s) = &self.sdc {
            out.push_str(&format!(
                "sdc     : {} clocks, {} input-delays, {} output-delays, {} exceptions\n",
                s.clocks.len(),
                s.input_delays.len(),
                s.output_delays.len(),
                s.exceptions.len()
            ));
        }
        if let Some(sp) = &self.spef {
            out.push_str(&format!("spef    : {} nets\n", sp.nets.len()));
        }
        if let Some(lf) = &self.lef {
            out.push_str(&format!("lef     : {} layers\n", lf.layers.len()));
        }
        if let Some(d) = &self.def {
            out.push_str(&format!(
                "def     : {} signal nets, {} power nets, {} components\n",
                d.nets.len(),
                d.power_nets.len(),
                d.comps.len()
            ));
        }
        if out.is_empty() {
            out.push_str("(empty design — nothing loaded)\n");
        }
        out
    }

    /// JSON summary (std-only, hand-rolled — matches the toolchain `--json` contract).
    pub fn to_json(&self) -> String {
        let mut parts: Vec<String> = Vec::new();
        match &self.netlist {
            Some(nl) => parts.push(format!(
                "\"netlist\":{{\"module\":{},\"inputs\":{},\"outputs\":{},\"instances\":{}}}",
                jstr(&nl.module),
                nl.inputs.len(),
                nl.outputs.len(),
                nl.insts.len()
            )),
            None => parts.push("\"netlist\":null".into()),
        }
        parts.push(format!(
            "\"liberty\":{{\"libraries\":{},\"cells\":{}}}",
            self.libs.len(),
            self.lib_cell_count()
        ));
        match &self.sdc {
            Some(s) => parts.push(format!(
                "\"sdc\":{{\"clocks\":{},\"input_delays\":{},\"output_delays\":{},\"exceptions\":{}}}",
                s.clocks.len(),
                s.input_delays.len(),
                s.output_delays.len(),
                s.exceptions.len()
            )),
            None => parts.push("\"sdc\":null".into()),
        }
        match &self.spef {
            Some(sp) => parts.push(format!("\"spef\":{{\"nets\":{}}}", sp.nets.len())),
            None => parts.push("\"spef\":null".into()),
        }
        match &self.lef {
            Some(lf) => parts.push(format!("\"lef\":{{\"layers\":{}}}", lf.layers.len())),
            None => parts.push("\"lef\":null".into()),
        }
        match &self.def {
            Some(d) => parts.push(format!(
                "\"def\":{{\"nets\":{},\"power_nets\":{},\"components\":{}}}",
                d.nets.len(),
                d.power_nets.len(),
                d.comps.len()
            )),
            None => parts.push("\"def\":null".into()),
        }
        let steps: Vec<String> = self
            .steps
            .iter()
            .map(|s| {
                format!(
                    "{{\"kind\":{},\"path\":{},\"summary\":{}}}",
                    jstr(s.kind),
                    jstr(&s.path),
                    jstr(&s.summary)
                )
            })
            .collect();
        parts.push(format!("\"steps\":[{}]", steps.join(",")));
        format!("{{{}}}", parts.join(","))
    }
}

/// Minimal JSON string escaper (std-only — no serde dependency).
fn jstr(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_design_summary_and_json() {
        let d = Design::new();
        assert!(d.summary().contains("empty"));
        assert!(d.to_json().contains("\"netlist\":null"));
        assert!(d.to_json().contains("\"steps\":[]"));
    }

    #[test]
    fn unknown_extension_is_an_error() {
        let mut d = Design::new();
        let err = d.load("design.gds").unwrap_err();
        assert!(err.to_string().contains("unknown extension"));
    }
}
