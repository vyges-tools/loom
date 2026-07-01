#!/usr/bin/env python3
"""Load a design into loom and print what it holds — the Python front door.

    python examples/inspect_design.py ../examples/top/top.v ../examples/top/cells.lib

With no arguments it loads the bundled three-inverter example.
"""

import os
import sys

import vyges_loom


def main(argv):
    files = argv[1:]
    if not files:
        ex = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "examples", "top"))
        files = [os.path.join(ex, f) for f in ("top.v", "cells.lib", "top.sdc", "top.spef")]

    d = vyges_loom.Design()
    for f in files:
        kind = d.load(f)
        print(f"loaded {kind:8} <- {f}")

    print(f"\nvyges-loom {vyges_loom.__version__}\n")
    print(d.summary())

    nl = d.netlist
    if nl is not None:
        print(f"module {nl.module}: {len(nl.inputs)} inputs, {len(nl.outputs)} outputs, {len(nl)} instances")
        for inst in nl.instances:
            pins = ", ".join(f"{p}={n}" for p, n in inst.connections)
            print(f"  {inst.cell:8} {inst.name:6} ({pins})")


if __name__ == "__main__":
    main(sys.argv)
