/////////////////////////////////////////////////////////////
// Created by: synthetic netlist writer (loom test fixture)
// Version   : 1.0
// Date      : not recorded
// Block Name: chip_top
/////////////////////////////////////////////////////////////
//
// Written for loom, in the style commercial netlist writers use for a mapped,
// whole-hierarchy netlist: a header comment block naming the block, every
// module of the design in one file, non-ANSI port lists, NO scalar wire
// declarations (only buses are declared), constant ties through `assign`, an
// inout pad, and library datapath modules that nothing instantiates.

module dp_sub_w4 ( A, B, DIFF );
  input [3:0] A;
  input [3:0] B;
  output [3:0] DIFF;

  XOR2_X1 U1 ( .A1(A[0]), .A2(B[0]), .Z(DIFF[0]) );
  XOR2_X1 U2 ( .A1(A[1]), .A2(B[1]), .Z(DIFF[1]) );
  XOR2_X1 U3 ( .A1(A[2]), .A2(B[2]), .Z(DIFF[2]) );
  XOR2_X1 U4 ( .A1(A[3]), .A2(B[3]), .Z(DIFF[3]) );
endmodule

module dp_add_w2 ( A, B, SUM );
  input [1:0] A;
  input [1:0] B;
  output [1:0] SUM;

  XOR2_X1 U1 ( .A1(A[0]), .A2(B[0]), .Z(SUM[0]) );
  XNOR2_X1 U2 ( .A1(A[1]), .A2(B[1]), .ZN(SUM[1]) );
endmodule

module io_pad ( PAD, DIN, DOUT, OEN );
  inout PAD;
  input DIN, OEN;
  output DOUT;

  BIDIR_PAD U0 ( .PAD(PAD), .I(DIN), .OEN(OEN), .C(DOUT) );
endmodule

module core_alu ( clk, a, b, sel, y );
  input clk, sel;
  input [1:0] a;
  input [1:0] b;
  output [1:0] y;

  NAND2_X1 U1 ( .A1(a[0]), .A2(b[0]), .ZN(n1) );
  NAND2_X1 U2 ( .A1(a[1]), .A2(b[1]), .ZN(n2) );
  MUX2_X1 U3 ( .I0(n1), .I1(n2), .S(sel), .Z(n3) );
  DFF_X1 y_reg_0_ ( .D(n3), .CK(clk), .Q(y[0]) );
  DFF_X1 y_reg_1_ ( .D(n1), .CK(clk), .Q(y[1]), .QN() );
endmodule

module chip_top ( clk, sel, a, b, y, pad_io, spare );
  input clk, sel;
  input [1:0] a;
  input [1:0] b;
  output [1:0] y;
  output spare;
  inout pad_io;
  wire   [1:0] y_int;

  assign tie_lo = 1'b0;

  core_alu u_core ( .clk(clk), .a(a), .b(b), .sel(n_sel), .y(y_int) );
  INV_X1 U10 ( .I(sel), .ZN(n_sel) );
  BUF_X1 U11 ( .I(y_int[0]), .Z(y[0]) );
  BUF_X1 U12 ( .I(y_int[1]), .Z(y[1]) );
  io_pad u_pad ( .PAD(pad_io), .DIN(y_int[0]), .DOUT(pad_in), .OEN(1'b0) );
  DFF_X1 pad_reg ( .D(pad_in), .CK(clk), .Q(n_pq) );
  AND2_X1 U13 ( .A1(n_pq), .A2(tie_lo), .Z(spare) );
endmodule
