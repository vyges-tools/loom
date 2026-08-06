// Parameter override whose value is an IDENTIFIER, not a literal.
//
// `#(.WIDTH(WIDTH))` cannot be told from a connection by looking at the value — the distinction
// is positional, which is why a ground truth that filtered on the value alone still counted it.
//
// Construct first seen in Yosys' test corpus (ISC), tests/techmap/lcu_refined.v.
module param_override_ident(P, G, CI, CO);
  parameter WIDTH = 4;
  input [WIDTH-1:0] P, G;
  input CI;
  output [WIDTH-1:0] CO;
  lcu_impl #(.WIDTH(WIDTH)) u_impl (.P(P), .G(G), .CI(CI), .CO(CO));
endmodule
