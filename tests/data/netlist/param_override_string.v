// Parameter override with a STRING value.
//
// `#(.MODE("AND"))` sits between the cell and the instance name. Read without allowing for it,
// `#` became the instance NAME, `.MODE` became its only connection, and the gate's three real
// connections were never seen.
//
// Construct first seen in Yosys' own test corpus (ISC), tests/arch/fabulous/custom_map.v.
// Our OpenLane netlists cannot contain it: synthesis flattens parameters away.
module param_override_string(A, B, Y);
  input [7:0] A, B;
  output [7:0] Y;
  ALU #(.MODE("AND")) u_alu (.A(A), .B(B), .Y(Y));
endmodule
