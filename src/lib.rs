//! vyges-loom — the shared design-data foundation (the *loom* the engines weave on).
//!
//! Loom is the **data plane** of the Vyges EDA toolchain: one set of
//! high-performance **parsers** for the standard formats, one in-memory
//! **design database** ([`Design`]) every engine queries, and one place the
//! **cross-step state** lives (the runtime sibling of `vyges-metadata.json`).
//!
//! It is *not* an engine and *not* an RTL elaborator — it is the parse-once,
//! query-many substrate. The sign-off/analysis engines (`vyges-char`,
//! `vyges-sta-si`, `vyges-extract`, `vyges-em-ir`, `vyges-power`, `vyges-lvs`)
//! build *on* loom; `vyges-sley` (the orchestrator) sequences them; `vyges`
//! (the CLI) is the front door.
//!
//! ## Seeded from `vyges-sta-si`
//!
//! The readers here ([`liberty`], [`netlist`], [`sdc`], [`spef`]) were promoted
//! from `vyges-sta-si`'s in-tree parsers — the most complete in the stack — so
//! every engine shares one implementation instead of re-rolling its own. The
//! SDC reader was decoupled from sta-si's job model: it now produces a
//! self-contained [`sdc::Sdc`] (with its own [`sdc::Exception`] type) that
//! engines apply to their own job model.
//!
//! ## Std-only
//!
//! No external dependencies — builds and tests offline. Standard file formats
//! are the plug boundary; loom is where they are parsed once into shared state.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_loom::…` paths resolve here.
extern crate self as vyges_loom;

pub mod liberty;
/// Liberty `function` expressions, parsed as the reference parses them (a logic checker reads a
/// cell's function from here).
pub mod liberty_func;
/// CCS (Composite Current Source) data model — the waveform groups the Liberty
/// reader populates (NLDM + CCS). The current-source delay *calculation* belongs
/// to the timing engine (`vyges-sta-si`); loom holds + can evaluate the curves.
pub mod ccs;
pub mod dmp;
/// Cross-process on-disk parse cache for `Lib` (behind `Lib::load_opts`; #38).
pub mod libcache;
pub mod netlist;
/// Yosys `write_json` netlist reader — an alternate front-end that maps onto the
/// same [`netlist::Netlist`] the structural-Verilog reader produces.
pub mod yosys_json;
/// SPICE **transistor-level** netlist reader (its own device-level `Netlist`, distinct
/// from the gate-level [`netlist`]) — hierarchical flatten + PDK device recognition.
/// The netlist front end LVS compares against an extracted layout.
pub mod spice;
pub mod sdc;
pub mod sdc_write;
/// Two SDC sets compared as constraints, not text: added, removed and changed, each loosening or
/// tightening.
pub mod sdc_diff;
/// SDC constraint linter — completeness and consistency of a design's constraints, from the
/// same SDC + netlist (+ Liberty) a timer loads (`vyges sdc check`; moved from `vyges-sta-si`).
pub mod sdc_lint;
/// Overlapping timing exceptions: which one wins on the paths they share, by the reference's
/// exception priority (`exception-shadowed`, `-conflict`, `-redundant` in the lint).
pub mod sdc_clock_relations;
pub mod sdc_genclk;
pub mod sdc_overlap;
pub mod sdc_path_fate;
/// Waivers for constraint findings, as a Tcl file of `waive_finding` commands: stable finding
/// ids, a template writer, the reader, and applying them (waived, active, stale).
pub mod sdc_waivers;
/// The `clock_domains` slice of `vyges-metadata.json` the constraint linter reads.
pub mod ipmeta;
mod sdc_query;
pub mod threads;
mod sdc_flags;
pub mod spef;
pub mod coverage;
/// EM geometry sidecar (per-segment layer/width/length) — the physical dimensions
/// standard SPEF omits but current-density sign-off needs. Companion to [`spef`].
pub mod emgeom;

/// Rectilinear polygon sets — boolean algebra over axis-aligned rectangles, and the outlines
/// (with holes) that fall out of it. Shared substrate: tap needs it for row-region boundaries,
/// pdn for strap-minus-blockage areas.
pub mod poly90;
/// KLayout net-dump reader — turns a headless-KLayout extraction dump into a
/// [`spef::Spef`] + [`emgeom::EmGeom`]. The data boundary for the RCX front-end.
pub mod klayout;
/// Simulation **activity** readers: [`vcd`] (VCD text) and [`saif`] (cumulative SAIF)
/// turn a waveform into per-net toggle counts, and [`names`] resolves them to netlist
/// nets (scope-aware, bit-level). The switching-activity source for `vyges-power` and
/// `vyges-em-ir`. (An FST reader lands behind the `fst` feature.)
pub mod names;
pub mod saif;
pub mod vcd;
/// FST (GTKWave binary waveform) activity reader — same `NetIndex` output as [`vcd`].
/// Behind the `fst` feature because it needs a zlib decoder (`miniz_oxide`); LZ4 is
/// clean-roomed in-tree. On by default (the same inflater `gzip` already pulls in);
/// `default-features = false` keeps loom's core std-only.
#[cfg(feature = "fst")]
pub mod fst;
/// The whole-file FST reader: hierarchy, every value change, reals and strings, streamed a
/// block at a time. ⛔ Query an FST in place; never convert it to VCD to inspect it.
#[cfg(feature = "fst")]
pub mod fst_wave;
/// The FST writer, and VCD → FST / FST-subset / bounded FST → VCD conversions.
#[cfg(feature = "fst")]
pub mod fst_writer;
/// `vyges loom fst`: header, hierarchy, transition and sample queries on an FST.
#[cfg(feature = "fst")]
pub mod fst_query;
#[cfg(feature = "fst")]
mod lz4;
/// Unified tech-LEF reader (per-layer width/thickness/resistance/EM limits) —
/// the superset of the extraction and PDN/EM views.
pub mod lef;
/// Unified DEF reader — signal `NETS` (µm, extraction) + `SPECIALNETS` power grid
/// (DBU, PDN) + `COMPONENTS`, parsed once.
pub mod def;
pub mod design;
/// Shared, std-only diagnostic-verbosity control (`-q`/`-v`, `VYGES_LOG`) — the
/// common CLI plumbing engines reuse, so every Vyges binary behaves identically.
pub mod verbosity;

pub use design::{Design, DesignError, Step};
/// Minimal std-only JSON reading, shared rather than reimplemented per engine.
pub use yosys_json::json;

/// Crate version (`CARGO_PKG_VERSION`).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Git commit baked at build time by `build.rs` (`-dirty` when the tree is dirty).
pub const GIT_SHA: &str = env!("VYGES_GIT_SHA");

pub const COPYRIGHT: &str = "© 2026 Vyges. All Rights Reserved.  https://vyges.com";

/// The `vyges-loom` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
