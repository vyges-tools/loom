//! Python bindings for `vyges-loom` — the parse-once / query-many design database.
//!
//! Exposes the loom `Design` (load the standard formats once, query the shared
//! in-memory database from Python) so notebooks, `cocotb`, and Python EDA flows can
//! drive the Vyges data plane directly instead of shelling out to a CLI.
//!
//! ```python
//! import vyges_loom
//! d = vyges_loom.Design()
//! d.load("top.v")        # structural Verilog netlist
//! d.load("cells.lib")    # Liberty
//! print(d.summary())
//! nl = d.netlist
//! print(nl.module, len(nl.instances))
//! ```
//!
//! This crate is intentionally separate from the loom core: the core stays
//! std-only and builds offline; only these optional bindings pull in PyO3.

use loom_core::{netlist, Design as CoreDesign};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// A single cell instance in the netlist.
#[pyclass(module = "vyges_loom")]
struct Inst {
    inner: netlist::Inst,
}

#[pymethods]
impl Inst {
    /// The library cell (master) this instance is of.
    #[getter]
    fn cell(&self) -> String {
        self.inner.cell.clone()
    }
    /// The instance name.
    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }
    /// The `(pin, net)` connections, in netlist order.
    #[getter]
    fn connections(&self) -> Vec<(String, String)> {
        self.inner.conns.clone()
    }

    fn __repr__(&self) -> String {
        format!(
            "Inst(cell={:?}, name={:?}, {} connection(s))",
            self.inner.cell,
            self.inner.name,
            self.inner.conns.len()
        )
    }
}

/// The structural netlist: module header, ports, and cell instances.
#[pyclass(module = "vyges_loom")]
struct Netlist {
    inner: netlist::Netlist,
}

#[pymethods]
impl Netlist {
    /// Top module name.
    #[getter]
    fn module(&self) -> String {
        self.inner.module.clone()
    }
    /// Input port names.
    #[getter]
    fn inputs(&self) -> Vec<String> {
        self.inner.inputs.clone()
    }
    /// Output port names.
    #[getter]
    fn outputs(&self) -> Vec<String> {
        self.inner.outputs.clone()
    }
    /// All cell instances.
    #[getter]
    fn instances(&self) -> Vec<Inst> {
        self.inner.insts.iter().cloned().map(|inner| Inst { inner }).collect()
    }
    /// Number of instances (also `len(netlist)`).
    fn __len__(&self) -> usize {
        self.inner.insts.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "Netlist(module={:?}, {} inputs, {} outputs, {} instances)",
            self.inner.module,
            self.inner.inputs.len(),
            self.inner.outputs.len(),
            self.inner.insts.len()
        )
    }
}

/// One recorded load — loom's cross-step provenance (`kind`, `path`, `summary`).
#[pyclass(module = "vyges_loom")]
struct Step {
    #[pyo3(get)]
    kind: String,
    #[pyo3(get)]
    path: String,
    #[pyo3(get)]
    summary: String,
}

#[pymethods]
impl Step {
    fn __repr__(&self) -> String {
        format!("Step(kind={:?}, path={:?}, summary={:?})", self.kind, self.path, self.summary)
    }
}

/// The shared in-memory design database. Load the standard formats once, then
/// query the one shared design.
#[pyclass(module = "vyges_loom")]
struct Design {
    inner: CoreDesign,
}

#[pymethods]
impl Design {
    #[new]
    fn new() -> Self {
        Design { inner: CoreDesign::new() }
    }

    /// Load a file, dispatching on extension (`.v`/`.sv`, `.lib`, `.sdc`, `.spef`,
    /// `.lef`/`.tlef`, `.def`). Returns the kind loaded (e.g. `"netlist"`). Raises
    /// `ValueError` on an unknown extension or a parse error.
    fn load(&mut self, path: &str) -> PyResult<String> {
        self.inner.load(path).map(|k| k.to_string()).map_err(to_py_err)
    }

    /// Load a structural Verilog netlist (`.v`/`.sv`).
    fn load_netlist(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_netlist(path).map_err(to_py_err)
    }
    /// Load a Liberty timing library (`.lib`). Appends — call once per corner.
    fn load_liberty(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_liberty(path).map_err(to_py_err)
    }
    /// Load SDC constraints (`.sdc`).
    fn load_sdc(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_sdc(path).map_err(to_py_err)
    }
    /// Load SPEF parasitics (`.spef`).
    fn load_spef(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_spef(path).map_err(to_py_err)
    }
    /// Load a tech-LEF (`.lef`/`.tlef`).
    fn load_lef(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_lef(path).map_err(to_py_err)
    }
    /// Load a DEF (`.def`).
    fn load_def(&mut self, path: &str) -> PyResult<()> {
        self.inner.load_def(path).map_err(to_py_err)
    }

    /// The netlist, or `None` if no netlist has been loaded.
    #[getter]
    fn netlist(&self) -> Option<Netlist> {
        self.inner.netlist.clone().map(|inner| Netlist { inner })
    }
    /// Total cells across all loaded Liberty libraries.
    #[getter]
    fn lib_cell_count(&self) -> usize {
        self.inner.lib_cell_count()
    }
    /// Number of Liberty libraries (corners) loaded.
    #[getter]
    fn liberty_count(&self) -> usize {
        self.inner.libs.len()
    }
    /// True if SDC constraints are loaded.
    #[getter]
    fn has_sdc(&self) -> bool {
        self.inner.sdc.is_some()
    }
    /// True if SPEF parasitics are loaded.
    #[getter]
    fn has_spef(&self) -> bool {
        self.inner.spef.is_some()
    }
    /// True if a tech-LEF is loaded.
    #[getter]
    fn has_lef(&self) -> bool {
        self.inner.lef.is_some()
    }
    /// True if a DEF is loaded.
    #[getter]
    fn has_def(&self) -> bool {
        self.inner.def.is_some()
    }
    /// Cross-step provenance: every load, in order.
    #[getter]
    fn steps(&self) -> Vec<Step> {
        self.inner
            .steps
            .iter()
            .map(|s| Step { kind: s.kind.to_string(), path: s.path.clone(), summary: s.summary.clone() })
            .collect()
    }

    /// Human-readable multi-line summary of what loom is holding.
    fn summary(&self) -> String {
        self.inner.summary()
    }
    /// The design as a JSON string (matches the toolchain `--json` contract).
    fn to_json(&self) -> String {
        self.inner.to_json()
    }

    fn __repr__(&self) -> String {
        let nl = match &self.inner.netlist {
            Some(n) => format!("module={:?}", n.module),
            None => "no netlist".into(),
        };
        format!("Design({nl}, {} liberty cells, {} steps)", self.inner.lib_cell_count(), self.inner.steps.len())
    }
}

fn to_py_err(e: loom_core::DesignError) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// The `vyges_loom` Python module.
#[pymodule]
fn vyges_loom(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", loom_core::VERSION)?;
    m.add("__git_sha__", loom_core::GIT_SHA)?;
    m.add_class::<Design>()?;
    m.add_class::<Netlist>()?;
    m.add_class::<Inst>()?;
    m.add_class::<Step>()?;
    Ok(())
}
