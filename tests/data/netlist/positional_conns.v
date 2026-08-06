// Connections given by POSITION rather than by name.
//
// The pin a position maps to lives in the cell's LEF/Liberty, not here — but the nets do, and
// they are what connectivity needs. Recording nothing left the instance in the graph with no
// edges at all.
module positional_conns(a, y);
  input a;
  output y;
  INV u1 (a, y);
endmodule
