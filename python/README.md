# vyges-loom (Python)

Python bindings for **vyges-loom** — the *parse-once / query-many* design-data
foundation of the Vyges EDA toolchain. Load the standard formats (structural Verilog,
Liberty, SDC, SPEF, LEF/DEF) **once** into one in-memory design and query it from
Python — so notebooks, `cocotb`, and Python EDA flows can drive the Vyges data plane
directly instead of shelling out to a CLI.

The Rust core is std-only and dependency-free; this wheel is a thin, optional binding
layer over it (built with [PyO3](https://pyo3.rs) + [maturin](https://www.maturin.rs)).

## Install

```sh
pip install vyges-loom
```

## Use

```python
import vyges_loom

d = vyges_loom.Design()
d.load("top.v")        # structural Verilog netlist
d.load("cells.lib")    # Liberty (dispatch is by extension)
d.load("top.sdc")      # SDC constraints
d.load("top.spef")     # SPEF parasitics

print(d.summary())     # human-readable
print(d.to_json())     # the toolchain --json contract, as a string

nl = d.netlist                       # None if no netlist loaded
print(nl.module, len(nl.instances))  # "top" 3
for inst in nl.instances:
    print(inst.cell, inst.name, dict(inst.connections))

d.lib_cell_count       # cells across all Liberty corners
d.has_sdc, d.has_spef  # what else is loaded
d.steps                # cross-step provenance: every load, in order
```

## API

| | |
| --- | --- |
| `Design()` | new, empty design |
| `.load(path) -> str` | load by extension (`.v`/`.sv`, `.lib`, `.sdc`, `.spef`, `.lef`/`.tlef`, `.def`); returns the kind; raises `ValueError` on unknown extension / parse error |
| `.load_netlist/liberty/sdc/spef/lef/def(path)` | explicit loaders |
| `.netlist` | `Netlist` or `None` — `.module`, `.inputs`, `.outputs`, `.instances`, `len()` |
| `.lib_cell_count`, `.liberty_count` | Liberty totals |
| `.has_sdc/.has_spef/.has_lef/.has_def` | presence flags |
| `.steps` | list of `Step(kind, path, summary)` |
| `.summary()`, `.to_json()` | text / JSON views |
| `Netlist.instances` | list of `Inst(cell, name, connections)` |

## Build from source

```sh
pip install maturin
cd python
maturin develop --release      # build + install into the current environment
pytest                         # run the test suite
```

The wheel is built `abi3-py39`, so one build serves CPython 3.9+.

## License

Apache-2.0. © 2026 Vyges. <https://vyges.com>
