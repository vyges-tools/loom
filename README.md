# vyges-loom

**The shared design-data foundation for the Vyges EDA toolchain — the *loom* the
engines weave on.** Parse-once / query-many: one set of readers for the standard
formats (structural Verilog **or Yosys JSON**, Liberty, SDC, SPEF, LEF/DEF) feeding
one in-memory **design database** that every engine queries, plus the **cross-step state** (provenance)
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

### Which cells can stand in for which — `equivalence_class`

A resize move (upsize a critical driver, downsize for hold or leakage, Vt-swap) is only safe if
the replacement computes the same thing. A physical database will refuse a swap whose *pins* do
not match, but nothing checks *function* — so the Liberty layer answers it:

```rust
let lib = vyges_loom::liberty::Lib::load("cells.lib")?;
lib.equivalence_class("INV_X2");   // the drive ladder, ranked by area: [INV_X1, INV_X2, INV_X4]
lib.upsize_candidates("INV_X2");   // [INV_X4]
lib.downsize_candidates("INV_X2"); // [INV_X1]
```

Equivalence is decided by how much the library actually tells us: **`cell_footprint`** first —
that is the vendor asserting the cells are drop-in — and otherwise identical pin names plus
identical output `function`. Sequential cells are matched by footprint only, since a flop's
behaviour lives in an `ff` group rather than a pin function.

A timing-only library (no `function`, no `cell_footprint`) reports no equivalents rather than
guessing. Same pins is not evidence: an inverter and a buffer are indistinguishable by shape,
and swapping one for the other silently inverts the design.

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

## FST waveforms — query in place

> ⛔ **Never convert an FST file to VCD to look at it.** A simulation's VCD is many times the
> size of its FST: it can fill the disk, and read into an AI agent's context it exhausts that
> too. Query the FST instead — `--info` reads only the header, `--list` only the hierarchy, and a
> time-window query only the blocks that window touches.

```sh
vyges-loom fst run.fst --info                          # writer, timescale, time range, counts
vyges-loom fst run.fst --list --scope top.core --depth 1
vyges-loom fst run.fst --start 1000 --end 2000 top.core.pc   # transitions in a window
vyges-loom fst run.fst --sample_at 1500,1800 --exact top.core.pc top.core.valid
vyges-loom fst run.fst --count top.core.pc --json
vyges-loom fst run.fst --batch queries.txt             # one query per line, one file open
vyges-loom fst-write run.vcd -o run.fst                # VCD -> FST (streamed)
vyges-loom fst-write run.fst -o win.fst --start 1000 --end 2000 top.core   # window + subset
```

- **Reader** (`fst_wave`): every block type — header, value-change blocks (all three position
  table forms; zlib, LZ4 and FastLZ chains), geometry, blackout, hierarchy (gzip, LZ4, LZ4-duo)
  and gzip-wrapped files — streamed one block at a time. Reals and strings are decoded too.
- **Query** (`fst`): the flag set and output of CoralNPU's `fst_query`, line for line: default
  `--limit 1000` with a notice on stderr when it stops, `--tail`, `--count`, `--sample_at`,
  `--regex`/`--exact`, `--batch` with a `--- [query end] ---` marker; `--json` on every query.
  Where that tool misreads a file (1-bit `x`/`z`, FastLZ chains, some zlib-compressed sections,
  gzip-wrapped files) this one follows the format, as `fst2vcd` does; it also prints reals and
  strings, which that tool skips.
- **Writer** (`fst_writer`): files read back identically by `fst2vcd`, `fst_query` and this
  reader. `fst-to-vcd` exists only for a tool that cannot read FST: it requires a time window
  **and** a signal list and stops at 256 MiB.
- The `fst` feature is on by default; `default-features = false` leaves it out. Value-change
  blocks of the two older types (1 and 5) are decoded but no corpus file exercises them yet.

## Use from Python

The same parse-once / query-many database is available to Python (notebooks,
`cocotb`, Python EDA flows) via optional bindings in [`python/`](python/). Until the
wheel is published to PyPI, build from source (`cd python && maturin develop`); the
published path will be:

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
| `yosys_json` | Yosys `write_json` netlist → same `netlist` model |
| `liberty` + `ccs` | Liberty timing/power libraries (NLDM + CCS data model) |
| `sdc` | SDC timing constraints (self-contained model) |
| `spef` | parasitic RC |
| `design` | the in-memory `Design` DB + cross-step state |
| `verbosity` | shared `-q`/`-v` / `VYGES_LOG` plumbing engines reuse |
| `vcd`, `saif`, `fst` | switching activity per net, for power |
| `fst_wave`, `fst_writer`, `fst_query` | FST waveforms: full reader, writer, the `fst` query command |

Seeded from `vyges-sta-si`'s in-tree parsers (the most complete in the stack); the
SDC reader was decoupled from sta-si's job model into a standalone `Sdc`.

## License

Apache-2.0. © 2026 Vyges. <https://vyges.com>
