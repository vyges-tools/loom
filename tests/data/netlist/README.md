# Netlist regression fixtures

Small structural-Verilog files, each pinning ONE construct that this reader once got wrong.
They are ours, written to isolate the construct; where a construct was first seen elsewhere the
file says so, because knowing which corpus exposed it is what tells you where to look next.

`tests/corpus.rs` reads this directory **unconditionally**, so these run in CI with no PDK, no
flow output and no `VYGES_CORPUS` — unlike the real-design corpora, which are opt-in.
