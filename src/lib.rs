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

pub mod liberty;
/// CCS (Composite Current Source) data model — the waveform groups the Liberty
/// reader populates (NLDM + CCS). The current-source delay *calculation* belongs
/// to the timing engine (`vyges-sta-si`); loom holds + can evaluate the curves.
pub mod ccs;
pub mod netlist;
/// Yosys `write_json` netlist reader — an alternate front-end that maps onto the
/// same [`netlist::Netlist`] the structural-Verilog reader produces.
pub mod yosys_json;
pub mod sdc;
pub mod spef;
/// Simulation **activity** readers: [`vcd`] (VCD text) and [`saif`] (cumulative SAIF)
/// turn a waveform into per-net toggle counts, and [`names`] resolves them to netlist
/// nets (scope-aware, bit-level). The switching-activity source for `vyges-power` and
/// `vyges-em-ir`. (An FST reader lands behind the `fst` feature.)
pub mod names;
pub mod saif;
pub mod vcd;
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

/// Crate version (`CARGO_PKG_VERSION`).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Git commit baked at build time by `build.rs` (`-dirty` when the tree is dirty).
pub const GIT_SHA: &str = env!("VYGES_GIT_SHA");

pub const COPYRIGHT: &str = "© 2026 Vyges. All Rights Reserved.  https://vyges.com";
