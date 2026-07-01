# vyges-loom

**The shared design-data foundation for the Vyges EDA toolchain — the *loom* the
engines weave on.** Parse-once / query-many: one set of readers for the standard
formats (structural Verilog, Liberty, SDC, SPEF) feeding one in-memory **design
database** that every engine queries, plus the **cross-step state** (provenance)
the orchestrator builds on.

Loom is the **data plane**. The sign-off/analysis engines (`vyges-char`,
`vyges-sta-si`, `vyges-extract`, `vyges-em-ir`, `vyges-power`, `vyges-lvs`,
`vyges-layout`) build *on* loom; `vyges-sley` sequences them; `vyges` (the CLI) is
the front door. Loom is **not** an engine and **not** an RTL elaborator.

Std-only Rust — builds and tests offline, runs on any modern silicon (Apple
Silicon / Graviton / x86_64), and you can experiment with GPUs too via [rust-gpu](https://rust-gpu.github.io/).

## Use as a library

```rust
use vyges_loom::Design;

let mut d = Design::new();
d.load("top.v")?;       // structural Verilog netlist
d.load("cells.lib")?;   // Liberty (NLDM + CCS)
d.load("top.sdc")?;     // constraints
d.load("top.spef")?;    // parasitics
// every engine queries the one shared design:
let nl = d.netlist.as_ref().unwrap();
println!("{} instances, {} cells", nl.insts.len(), d.lib_cell_count());
```

## Use as a CLI (common, design-wide commands)

```sh
vyges-loom inspect top.v cells.lib top.sdc top.spef   # parse + summarize
vyges-loom inspect --json top.v cells.lib             # machine-readable
vyges-loom check top.v cells.lib                       # parse-validate (CI gate)
vyges-loom --version                                   # vyges-loom <ver> (<git-sha>)
```

Common commands live here; **tool-specific verbs (timing, power, extraction, …)
belong to the engines**, which build on the `vyges_loom` library and — in the
two-utility packaging — attach as `vyges-loom <engine>` subcommands.

## Use from Python

The same parse-once / query-many database is available to Python (notebooks,
`cocotb`, Python EDA flows) via optional bindings in [`python/`](python/):

```sh
pip install vyges-loom
```

```python
import vyges_loom
d = vyges_loom.Design()
d.load("top.v"); d.load("cells.lib")   # dispatch is by extension
print(d.netlist.module, len(d.netlist.instances), d.lib_cell_count)
```

The Rust core stays std-only and dependency-free; the bindings crate (PyO3 + maturin,
built `abi3-py39`) is a thin, additive layer over it — see [`python/README.md`](python/README.md).

## Modules

| Module | What it reads / holds |
| --- | --- |
| `netlist` | structural (gate-level) Verilog |
| `liberty` + `ccs` | Liberty timing/power libraries (NLDM + CCS data model) |
| `sdc` | SDC timing constraints (self-contained model) |
| `spef` | parasitic RC |
| `design` | the in-memory `Design` DB + cross-step state |
| `verbosity` | shared `-q`/`-v` / `VYGES_LOG` plumbing engines reuse |

Seeded from `vyges-sta-si`'s in-tree parsers (the most complete in the stack); the
SDC reader was decoupled from sta-si's job model into a standalone `Sdc`.

## License

Apache-2.0. © 2026 Vyges. <https://vyges.com>
