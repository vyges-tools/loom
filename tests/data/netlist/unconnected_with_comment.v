// A pin left unconnected, with a comment INSIDE the parentheses.
//
// `.out2 (/* unconnected */)` connects nothing. The comment must not be read as a net, and the
// pin must not be recorded — a dangling pin recorded as connected is a false edge.
//
// Construct first seen in Yosys' test corpus (ISC), tests/cxxrtl/test_unconnected_output.v.
module unconnected_with_comment(clk, in, out);
  input clk, in;
  output out;
  blackbox u_bb (
    .clk  (clk),
    .in   (in),
    .out1 (out),
    .out2 (/* unconnected */)
  );
endmodule
