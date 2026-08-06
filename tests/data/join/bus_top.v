// A design with a bus, so the netlist names nets `d[0]` … `q[3]`.
// Paired with bus_top.vcd, which dumps the same run. The two must agree about those names.
module bus_top ( clk, d, q );
  input        clk;
  input  [3:0] d;
  output [3:0] q;
  DFF r0 ( .CLK(clk), .D(d[0]), .Q(q[0]) );
  DFF r1 ( .CLK(clk), .D(d[1]), .Q(q[1]) );
  DFF r2 ( .CLK(clk), .D(d[2]), .Q(q[2]) );
  DFF r3 ( .CLK(clk), .D(d[3]), .Q(q[3]) );
endmodule
