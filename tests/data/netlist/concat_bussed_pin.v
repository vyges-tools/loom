// A concatenation on a bussed pin, with escaped identifiers as members.
//
// `.D({a, b, c})` is three connections, MSB-first — D[2], D[1], D[0] — which is how Liberty
// names the bits of such a pin. Keeping only the first member left a net literally called `{a`
// and lost the rest; one wrapper in this organisation has 101 of them.
module concat_bussed_pin(a, b, y);
  input a, b;
  output y;
  wire \esc.net[1] , \esc.net[0] ;
  MACRO u_macro (.D({a, b}), .E({\esc.net[1] , \esc.net[0] }), .Y(y));
endmodule
