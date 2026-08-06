//! Robustness for the structural-Verilog reader.
//!
//! This parser produces **connectivity**, which fails differently from every other reader in
//! this crate. A missing capacitance shows up as a zero; a missing *instance* or a mangled net
//! name shows up as a different circuit, and nothing downstream can tell it from the real one.
//!
//! So the governing property here is **conservation**: every `.pin(net)` in the file becomes
//! exactly one connection in the result. That is checkable against any netlist from any tool
//! without a golden answer, and it is what `tests/corpus.rs` runs over real ones.
//!
//! Every case below was a silent defect before it was a test, and each was found by reading a
//! real gate-level netlist rather than by imagining what one contains.

use vyges_loom::netlist::{self, Netlist};

fn parse(src: &str) -> Netlist {
    netlist::parse(src).expect("valid structural Verilog")
}

fn conns_of<'a>(n: &'a Netlist, inst: &str) -> Vec<(&'a str, &'a str)> {
    n.insts
        .iter()
        .find(|i| i.name == inst)
        .map(|i| i.conns.iter().map(|(p, x)| (p.as_str(), x.as_str())).collect())
        .unwrap_or_default()
}

#[test]
fn the_known_good_netlist_reads_as_expected() {
    let n = parse(
        "module m(a, y);\n input a;\n output y;\n wire w;\n \
         INV u1 (.A(a), .Y(w));\n BUF u2 (.A(w), .Y(y));\nendmodule\n",
    );
    assert_eq!(n.module, "m");
    assert_eq!(n.inputs, ["a"]);
    assert_eq!(n.outputs, ["y"]);
    assert_eq!(n.insts.len(), 2);
    assert_eq!(conns_of(&n, "u1"), [("A", "a"), ("Y", "w")]);
    assert!(n.health().is_none(), "{:?}", n.health());
}

/// **A comment must not become a circuit.** `/* ... */` spans lines and is where tools park
/// disabled code; tokenized as if it were live, `/* INV bogus (.A(a)); */` became a real
/// instance — connectivity invented out of a comment.
#[test]
fn block_comments_do_not_create_instances() {
    let n = parse(
        "module m(a, y);\n input a;\n output y;\n\
         /* INV bogus (.A(a), .Y(y));\n   AND also_bogus (.A(a), .B(a), .X(y)); */\n\
         INV u1 (.A(a), .Y(y));\nendmodule\n",
    );
    assert_eq!(n.insts.len(), 1, "{:?}", n.insts.iter().map(|i| &i.name).collect::<Vec<_>>());
    assert_eq!(n.insts[0].name, "u1");

    // an unterminated comment runs to end of file, as Verilog says
    let n = parse("module m(a); input a; INV u1 (.A(a)); /* trailing\nendmodule\n");
    assert_eq!(n.insts.len(), 1);

    // and `//` still works, including after real code on the same line
    let n = parse("module m(a); input a; INV u1 (.A(a)); // INV ghost (.A(a));\nendmodule\n");
    assert_eq!(n.insts.len(), 1);
}

/// **Top is the module nothing instantiates**, not the first one in the file. A hierarchical
/// netlist that lists its leaves first otherwise hands back a leaf as the design — silently,
/// with a plausible-looking port list.
#[test]
fn the_top_module_is_the_one_nothing_instantiates() {
    let leaf_first = "module sub(a, y); input a; output y; INV u0 (.A(a), .Y(y)); endmodule\n\
                      module top(a, y); input a; output y; sub u1 (.A(a), .Y(y)); endmodule\n";
    let n = parse(leaf_first);
    assert_eq!(n.module, "top", "leaf declared first must not win");
    assert_eq!(n.modules, ["sub", "top"]);
    assert!(n.health().unwrap().contains("2 modules"));

    // and the other order gives the same answer
    let top_first = "module top(a, y); input a; output y; sub u1 (.A(a), .Y(y)); endmodule\n\
                     module sub(a, y); input a; output y; INV u0 (.A(a), .Y(y)); endmodule\n";
    assert_eq!(parse(top_first).module, "top");

    // a single module is still the top, and reports nothing
    let one = parse("module only(a); input a; INV u (.A(a)); endmodule\n");
    assert_eq!(one.module, "only");
    assert!(one.health().is_none());
}

/// **`assign` is connectivity, not decoration.** Tools emit it for port aliases and constant
/// ties; dropping it leaves the left-hand side with no driver at all.
#[test]
fn assign_aliases_are_recorded() {
    let n = parse(
        "module m(a, y);\n input a;\n output y;\n wire w;\n \
         INV u1 (.A(a), .Y(w));\n assign y = w;\nendmodule\n",
    );
    assert_eq!(n.aliases, [("y".to_string(), "w".to_string())]);
    assert!(n.health().unwrap().contains("alias"));

    // bit-selects survive on both sides
    let n = parse("module m(a, y); input [1:0] a; output y; assign y = a[1]; endmodule\n");
    assert_eq!(n.aliases, [("y".to_string(), "a[1]".to_string())]);
}

/// **Positional connections carry no pin name**, but they do carry nets — and the nets are what
/// connectivity needs. Recording nothing left the instance in the graph with no edges at all.
#[test]
fn positional_connections_keep_their_nets() {
    let n = parse("module m(a, y);\n input a;\n output y;\n INV u1 (a, y);\nendmodule\n");
    assert_eq!(n.insts.len(), 1);
    let c = conns_of(&n, "u1");
    assert_eq!(c.len(), 2, "both nets recorded: {c:?}");
    assert_eq!(c.iter().map(|(_, x)| *x).collect::<Vec<_>>(), ["a", "y"]);
    assert!(c.iter().all(|(p, _)| p.is_empty()), "the pin name is not in the netlist");
    assert_eq!(n.positional, 2);
    assert!(n.health().unwrap().contains("by position"));

    // a named netlist reports no positional connections
    let named = parse("module m(a); input a; INV u1 (.A(a)); endmodule\n");
    assert_eq!(named.positional, 0);
}

/// **A concatenation on a bussed pin is several connections**, MSB-first — which is how Liberty
/// names the bits of such a pin. Keeping only the first member left a net literally called
/// `{a` and lost the rest; a real wrapper in this organisation has 101 of them.
#[test]
fn a_concatenation_expands_msb_first() {
    let n = parse("module m(a, b, c, y); input a, b, c; output y; \
                   MACRO u1 (.D({a, b, c}), .Y(y)); endmodule\n");
    let c = conns_of(&n, "u1");
    assert_eq!(
        c,
        [("D[2]", "a"), ("D[1]", "b"), ("D[0]", "c"), ("Y", "y")],
        "got {c:?}"
    );
    assert!(!c.iter().any(|(_, x)| x.contains('{') || x.contains('}')), "no brace leaked: {c:?}");

    // members may themselves be bit-selects
    let n = parse("module m(d, y); input [1:0] d; output y; \
                   MACRO u1 (.D({d[1], d[0]}), .Y(y)); endmodule\n");
    assert_eq!(conns_of(&n, "u1"), [("D[1]", "d[1]"), ("D[0]", "d[0]"), ("Y", "y")]);
}

/// Escaped identifiers run to whitespace and may contain `.`, `[`, `]` — yosys emits them for
/// hierarchical names. Splitting one leaves a fragment that mis-parses as another instance.
///
/// The NAME excludes the leading backslash, which introduces the identifier and is no more part
/// of it than the terminating whitespace: per the LRM `\foo ` denotes the same identifier as
/// `foo`. Keeping it made every hierarchical net fail to match the SPEF and DEF that describe
/// the same design — 767 of 14238 nets on a real block, and the 4527 coupling references a
/// timer then dropped in silence.
#[test]
fn escaped_identifiers_survive_intact() {
    let n = parse(
        "module m(a, y);\n input a;\n output y;\n \
         INV \\u_cpu.buf[0] (.A(a), .Y(\\u_cpu.net[3] ));\nendmodule\n",
    );
    assert_eq!(n.insts.len(), 1);
    assert_eq!(n.insts[0].name, "u_cpu.buf[0]");
    assert_eq!(conns_of(&n, "u_cpu.buf[0]"), [("A", "a"), ("Y", "u_cpu.net[3]")]);
}

/// Port bus ranges expand to bits so they match the bit-nets gates drive, and a bit-select
/// connection reassembles to the same spelling.
#[test]
fn buses_expand_and_bit_selects_reassemble() {
    let n = parse(
        "module m(d, y);\n input [1:0] d;\n output [3:2] y;\n \
         AND u1 (.A(d[0]), .B(d[1]), .X(y[3]));\nendmodule\n",
    );
    assert_eq!(n.inputs, ["d[1]", "d[0]"]);
    assert_eq!(n.outputs, ["y[3]", "y[2]"]);
    assert_eq!(conns_of(&n, "u1"), [("A", "d[0]"), ("B", "d[1]"), ("X", "y[3]")]);
}

/// **Conservation.** Every `.pin(net)` becomes exactly one connection — the property that makes
/// a netlist reader checkable against any file without a golden answer.
#[test]
fn every_named_connection_is_conserved() {
    let mut src = String::from("module m(a, y);\n input a;\n output y;\n");
    let mut want = 0;
    for i in 0..200 {
        src.push_str(&format!(" INV u{i} (.A(n{i}), .Y(n{}));\n", i + 1));
        want += 2;
    }
    // a filler cell with no pins at all, as a placed netlist is full of
    src.push_str(" FILL f0 ();\n");
    src.push_str("endmodule\n");
    let n = parse(&src);
    let got: usize = n.insts.iter().map(|i| i.conns.len()).sum();
    assert_eq!(got, want, "{want} connections in the text, {got} parsed");
    assert_eq!(n.insts.len(), 201, "the filler is an instance too");
}

/// Constants are not nets: `1'b0` is a tie, and recording it as a net named `1'b0` would make
/// every tied pin in the design look mutually connected.
#[test]
fn constants_are_not_recorded_as_nets() {
    let n = parse("module m(y); output y; TIE u1 (.A(1'b0), .B(1'b1), .Y(y)); endmodule\n");
    assert_eq!(conns_of(&n, "u1"), [("Y", "y")]);
}

/// **Handed RTL, this reader does not fail — it returns fragments.** Measured on one real file:
/// 2 instances recovered from a module carrying 4 558 connections, with no complaint. It is a
/// structural reader and that is a fair contract, but the caller has to be able to find out.
#[test]
fn behavioural_verilog_is_reported_as_not_a_netlist() {
    let rtl = "module m(clk, d, q);\n input clk, d;\n output q;\n reg q;\n \
               always @(posedge clk) begin\n   q <= d;\n end\nendmodule\n";
    let n = parse(rtl);
    assert!(n.behavioural > 0, "an always block must be noticed");
    let h = n.health().expect("must report");
    assert!(h.contains("RTL"), "{h}");
    assert!(h.contains("not a circuit"), "{h}");

    // a gate-level netlist says nothing
    let gl = parse("module m(a, y); input a; output y; INV u1 (.A(a), .Y(y)); endmodule\n");
    assert_eq!(gl.behavioural, 0);
    assert!(gl.health().is_none());
}

/// A pin connected to nothing (`.pin()`) and a pin tied to a sized constant (`.pin(32'h0)`) are
/// both real forms in a structural wrapper, and neither is a net. Recording either would make
/// every tied or dangling pin in the design look mutually connected.
#[test]
fn empty_and_constant_connections_are_not_nets() {
    let n = parse(
        "module m(y);\n output y;\n \
         CPU u1 (.boot_addr_i(32'h00008000), .irq_i(1'b0), .sleep_o(), .q(y));\nendmodule\n",
    );
    assert_eq!(conns_of(&n, "u1"), [("q", "y")], "only the real net survives");
}

/// **A parameter override sits between the cell and the instance name.** Yosys emits
/// `ALU #(.MODE("AND")) u1 (.A(a), .Y(y));` and so does any parameterised macro. Read without
/// allowing for it, `#` became the instance NAME, the parameters became its connections, and
/// the gate's real connections were never seen — a whole instance replaced by its own
/// parameters, silently.
///
/// Found by running this reader over Yosys' own test corpus (ISC), which contains forms our
/// flattened OpenLane netlists never do.
#[test]
fn a_parameter_override_does_not_swallow_the_instance() {
    let n = parse(
        "module m(a, b, y);\n input [7:0] a, b;\n output [7:0] y;\n \
         ALU #(.MODE(\"AND\")) u1 (.A(a), .B(b), .Y(y));\nendmodule\n",
    );
    assert_eq!(n.insts.len(), 1);
    assert_eq!(n.insts[0].cell, "ALU");
    assert_eq!(n.insts[0].name, "u1", "the instance name, not `#`");
    assert_eq!(conns_of(&n, "u1"), [("A", "a"), ("B", "b"), ("Y", "y")]);

    // multi-line, multiple parameters, and a trailing comma in the port list
    let n = parse(
        "module top(i, o);\n input [3:0] i;\n output [3:0] o;\n \
         python_inv #(\n   .width(4),\n   .depth(2)\n ) inv (\n  .i(i),\n  .o(o),\n );\nendmodule\n",
    );
    assert_eq!(n.insts.len(), 1);
    assert_eq!((n.insts[0].cell.as_str(), n.insts[0].name.as_str()), ("python_inv", "inv"));
    assert_eq!(conns_of(&n, "inv"), [("i", "i"), ("o", "o")]);

    // a positional parameter override, which is also legal
    let n = parse("module m(a, y); input a; output y; DLY #(3) u1 (.A(a), .Y(y)); endmodule\n");
    assert_eq!(n.insts[0].name, "u1");
    assert_eq!(conns_of(&n, "u1"), [("A", "a"), ("Y", "y")]);
}
