// Two modules, the LEAF declared first.
//
// Top is the module nothing instantiates. Taking the first one instead handed back `sub` — a
// leaf — as the design, with a plausible port list and no complaint.
module sub(a, y);
  input a;
  output y;
  INV u0 (.A(a), .Y(y));
endmodule

module top_leaf_first(a, y);
  input a;
  output y;
  sub u_sub (.A(a), .Y(y));
endmodule
