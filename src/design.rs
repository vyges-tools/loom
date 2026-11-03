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

use crate::{def, lef, liberty, netlist, sdc, spef, yosys_json};

/// One recorded load/step — the seed of loom's cross-step state (provenance).
#[derive(Debug, Clone)]
pub struct Step {
    /// `"netlist" | "flatten" | "liberty" | "sdc" | "spef" | "lef" | "def"`.
    pub kind: &'static str,
    pub path: String,
    /// Human-readable one-liner (counts).
    pub summary: String,
}

/// The shared in-memory design. Engines read from this rather than re-parsing.
#[derive(Debug, Default)]
pub struct Design {
    /// The netlist's TOP, owned — or, after [`Design::flatten_netlist`], the whole flat design.
    ///
    /// ⛔ On a hierarchical netlist the top alone is not the design (see
    /// [`netlist::Netlist::health`]); [`Design::hierarchy`] has every module.
    pub netlist: Option<netlist::Netlist>,
    /// EVERY module of the Verilog netlist loaded, in the compact form — or, once flattened, the
    /// flat design. Never only the top. `None` for a netlist read from Yosys JSON.
    pub hierarchy: Option<netlist::CompactDesign>,
    /// The top module of the next Verilog netlist loaded, as `link_design <top>` names it. `None`:
    /// the module nothing instantiates — and a netlist with more than one such is refused.
    pub netlist_top: Option<String>,
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

    /// Load a structural Verilog netlist (`.v`/`.sv`, gzipped or not): every module is kept in
    /// [`Design::hierarchy`], and the top in [`Design::netlist`]. A hierarchical netlist is
    /// flattened with [`Design::flatten_netlist`].
    pub fn load_netlist(&mut self, path: &str) -> Result<(), DesignError> {
        let all = netlist::load_compact_opt(path, self.netlist_top.as_deref()).map_err(|e| DesignError(e.to_string()))?;
        let nl = all.top_netlist();
        let summary = format!(
            "module {} — {} inputs, {} outputs, {} instances ({} module(s) in the file)",
            nl.module,
            nl.inputs.len(),
            nl.outputs.len(),
            nl.insts.len(),
            all.modules.len()
        );
        self.steps.push(Step { kind: "netlist", path: path.into(), summary });
        self.netlist = Some(nl);
        self.hierarchy = Some(all);
        Ok(())
    }

    /// Flatten the loaded Verilog netlist into ONE module (see
    /// [`netlist::CompactDesign::flatten`]): [`Design::netlist`] and [`Design::hierarchy`] then
    /// hold the flat design. `keep` names modules kept as blocks. Cells are checked against the
    /// Liberty libraries loaded so far; with none loaded, every cell not defined in the netlist
    /// is reported.
    pub fn flatten_netlist(&mut self, keep: &[String]) -> Result<netlist::FlattenReport, DesignError> {
        let Some(all) = self.hierarchy.as_ref() else {
            return Err(DesignError("flatten: no Verilog netlist loaded".into()));
        };
        let library_cells = (!self.libs.is_empty())
            .then(|| self.libs.iter().flat_map(|l| l.cells.keys().cloned()).collect());
        let opts = netlist::FlattenOptions { keep: keep.to_vec(), library_cells };
        let (flat, report) = all.flatten(&opts);
        let nl = flat.top_netlist();
        let mut summary = format!(
            "flattened {} — {} leaf instances from {} module instances",
            report.top, report.leaves, report.expanded
        );
        if let Some(h) = report.health() {
            summary.push_str(&format!("; {h}"));
        }
        let path = self.steps.iter().rev().find(|s| s.kind == "netlist").map(|s| s.path.clone()).unwrap_or_default();
        self.steps.push(Step { kind: "flatten", path, summary });
        self.netlist = Some(nl);
        self.hierarchy = Some(flat);
        Ok(report)
    }

    /// Load a Yosys JSON netlist (`.json`, from `write_json`). Sets the netlist,
    /// like [`Design::load_netlist`] but from Yosys's JSON instead of Verilog.
    pub fn load_netlist_json(&mut self, path: &str) -> Result<(), DesignError> {
        let nl = yosys_json::load(path).map_err(|e| DesignError(e.to_string()))?;
        self.set_netlist_json(path, nl);
        Ok(())
    }

    fn set_netlist_json(&mut self, path: &str, nl: netlist::Netlist) {
        let summary = format!(
            "module {} — {} inputs, {} outputs, {} instances",
            nl.module,
            nl.inputs.len(),
            nl.outputs.len(),
            nl.insts.len()
        );
        self.steps.push(Step { kind: "netlist", path: path.into(), summary });
        self.netlist = Some(nl);
        self.hierarchy = None;
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
        self.set_sdc(path, s);
        Ok(())
    }

    fn set_sdc(&mut self, path: &str, s: sdc::Sdc) {
        let summary = format!(
            "{} clocks, {} input-delays, {} output-delays, {} exceptions",
            s.clocks.len(),
            s.input_delays.len(),
            s.output_delays.len(),
            s.exceptions.len()
        );
        self.steps.push(Step { kind: "sdc", path: path.into(), summary });
        self.sdc = Some(s);
    }

    /// Load SPEF parasitics (`.spef`).
    pub fn load_spef(&mut self, path: &str) -> Result<(), DesignError> {
        let sp = spef::Spef::load(path).map_err(|e| DesignError(e.to_string()))?;
        self.set_spef(path, sp);
        Ok(())
    }

    fn set_spef(&mut self, path: &str, sp: spef::Spef) {
        let summary = format!("{} nets", sp.nets.len());
        self.steps.push(Step { kind: "spef", path: path.into(), summary });
        self.spef = Some(sp);
    }

    /// Load a tech-LEF (`.lef`/`.tlef`).
    pub fn load_lef(&mut self, path: &str) -> Result<(), DesignError> {
        let lf = lef::Lef::load(path).map_err(|e| DesignError(e.to_string()))?;
        self.set_lef(path, lf);
        Ok(())
    }

    fn set_lef(&mut self, path: &str, lf: lef::Lef) {
        let summary = format!("{} layers", lf.layers.len());
        self.steps.push(Step { kind: "lef", path: path.into(), summary });
        self.lef = Some(lf);
    }

    /// Load a DEF (`.def`).
    pub fn load_def(&mut self, path: &str) -> Result<(), DesignError> {
        let d = def::Def::load(path).map_err(|e| DesignError(e.to_string()))?;
        self.set_def(path, d);
        Ok(())
    }

    fn set_def(&mut self, path: &str, d: def::Def) {
        let summary = format!(
            "{} signal nets, {} power nets, {} components",
            d.nets.len(),
            d.power_nets.len(),
            d.comps.len()
        );
        self.steps.push(Step { kind: "def", path: path.into(), summary });
        self.def = Some(d);
    }

    /// Dispatch a file to the right reader by extension. Returns the `kind`
    /// loaded. Unknown extensions are an error (loom is explicit, no silent skip).
    ///
    /// A trailing `.gz` is stripped before the dispatch, and the file is inflated as it is read:
    /// `.v.gz`, `.lib.gz`, `.spef.gz`, `.json.gz`, `.sdc.gz`, `.lef.gz`, `.def.gz`. Field
    /// netlists, parasitics and advanced-node Liberty are shipped that way.
    pub fn load(&mut self, path: &str) -> Result<&'static str, DesignError> {
        let lower = path.to_ascii_lowercase();
        let gz = lower.ends_with(".gz");
        let inner = if gz { &path[..path.len() - 3] } else { path };
        let ext = std::path::Path::new(inner)
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        // the readers that take a path and read it whole; given a `.gz`, the inflated text
        let text = || netlist::read_text(path).map_err(|e| DesignError(e.to_string()));
        match ext.as_str() {
            // the netlist and Liberty readers inflate a gzip stream themselves
            "v" | "sv" | "vg" => self.load_netlist(path).map(|_| "netlist"),
            "lib" => self.load_liberty(path).map(|_| "liberty"),
            "json" if gz => {
                let nl = yosys_json::parse(&text()?).map_err(|e| DesignError(format!("{path}: {e}")))?;
                self.set_netlist_json(path, nl);
                Ok("netlist")
            }
            "json" => self.load_netlist_json(path).map(|_| "netlist"),
            "sdc" if gz => {
                let s = sdc::Sdc::parse(&text()?).map_err(|e| DesignError(format!("{path}: {e}")))?;
                self.set_sdc(path, s);
                Ok("sdc")
            }
            "sdc" => self.load_sdc(path).map(|_| "sdc"),
            "spef" if gz => {
                self.set_spef(path, spef::Spef::parse(&text()?));
                Ok("spef")
            }
            "spef" => self.load_spef(path).map(|_| "spef"),
            "lef" | "tlef" if gz => {
                let lf = lef::Lef::parse(&text()?).map_err(|e| DesignError(format!("{path}: {e}")))?;
                self.set_lef(path, lf);
                Ok("lef")
            }
            "lef" | "tlef" => self.load_lef(path).map(|_| "lef"),
            "def" if gz => {
                let d = def::Def::parse(&text()?).map_err(|e| DesignError(format!("{path}: {e}")))?;
                self.set_def(path, d);
                Ok("def")
            }
            "def" => self.load_def(path).map(|_| "def"),
            other => Err(DesignError(format!(
                "{path}: unknown extension '.{other}' (expected .v/.sv, .json, .lib, .sdc, .spef, .lef, \
                 .def, each optionally .gz)"
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

    /// The design holds every module of a hierarchical netlist, never only the top, and
    /// `flatten_netlist` replaces both views with the flat design, checking cells against the
    /// libraries loaded.
    #[test]
    fn a_hierarchical_netlist_is_held_whole_and_flattens() {
        let dir = std::env::temp_dir().join(format!("loom-design-flat-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let v = dir.join("h.v");
        std::fs::write(
            &v,
            "module sub (a, y); input a; output y; INV u (.A(a), .Y(y)); endmodule\n\
             module top (a, y); input a; output y; wire n; sub s0 (.a(a), .y(n)); sub s1 (.a(n), .y(y)); endmodule\n",
        )
        .unwrap();
        let mut d = Design::new();
        assert!(d.flatten_netlist(&[]).is_err(), "nothing to flatten yet");
        d.load(v.to_str().unwrap()).unwrap();
        assert_eq!(d.netlist.as_ref().unwrap().insts.len(), 2, "the top's own instances");
        assert_eq!(d.hierarchy.as_ref().unwrap().modules.len(), 2, "and every module");
        let rep = d.flatten_netlist(&[]).unwrap();
        assert_eq!(rep.undefined_cells, [("INV".to_string(), 2)], "no library: reported");
        let nl = d.netlist.as_ref().unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["s0/u", "s1/u"]);
        assert_eq!(d.hierarchy.as_ref().unwrap().modules.len(), 1, "the flat design");
        assert_eq!(d.steps.last().unwrap().kind, "flatten");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A trailing `.gz` is stripped before the dispatch, and the file inflated as it is read —
    /// on by default. (`design.v.gz` was "unknown extension '.gz'".)
    #[cfg(feature = "gzip")]
    #[test]
    fn a_gzipped_file_dispatches_on_the_extension_under_gz() {
        fn gz(text: &str) -> Vec<u8> {
            let mut g = vec![0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 0, 3];
            g.extend(miniz_oxide::deflate::compress_to_vec(text.as_bytes(), 6));
            g.extend_from_slice(&[0; 4]); // CRC-32 (not checked)
            g.extend_from_slice(&(text.len() as u32).to_le_bytes());
            g
        }
        let dir = std::env::temp_dir().join(format!("loom-design-gz-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let write = |name: &str, text: &str| {
            let p = dir.join(name);
            std::fs::write(&p, gz(text)).unwrap();
            p.to_str().unwrap().to_string()
        };
        let mut d = Design::new();
        let v = write("top.v.gz", "module top (a, y); input a; output y; INV u (.A(a), .Y(y)); endmodule\n");
        assert_eq!(d.load(&v).unwrap(), "netlist");
        assert_eq!(d.netlist.as_ref().unwrap().insts.len(), 1);
        let lib = write("cells.LIB.GZ", "library (l) { cell (INV) { pin (A) { direction : input; } } }\n");
        assert_eq!(d.load(&lib).unwrap(), "liberty");
        assert!(d.libs[0].cells.contains_key("INV"));
        let sp = write("top.spef.gz", "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"top\"\n*T_UNIT 1 NS\n*C_UNIT 1 PF\n*R_UNIT 1 OHM\n*L_UNIT 1 HENRY\n*D_NET a 0.5\n*CONN\n*P a I\n*END\n");
        assert_eq!(d.load(&sp).unwrap(), "spef");
        assert_eq!(d.spef.as_ref().unwrap().nets.len(), 1);
        let sdc = write("top.sdc.gz", "create_clock -name c -period 10 [get_ports a]\n");
        assert_eq!(d.load(&sdc).unwrap(), "sdc");
        assert_eq!(d.sdc.as_ref().unwrap().clocks.len(), 1);
        let err = d.load(&write("x.gds.gz", "")).unwrap_err();
        assert!(err.to_string().contains("unknown extension '.gds'"), "{err}");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn unknown_extension_is_an_error() {
        let mut d = Design::new();
        let err = d.load("design.gds").unwrap_err();
        assert!(err.to_string().contains("unknown extension"));
    }
}
