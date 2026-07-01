"""End-to-end tests for the vyges_loom Python bindings.

Runs against the same fixtures as the Rust integration test
(``examples/top``: structural Verilog + Liberty + SDC + SPEF), so the two
front-ends are validated on identical data.
"""

import json
import os

import pytest

import vyges_loom

HERE = os.path.dirname(__file__)
EX = os.path.normpath(os.path.join(HERE, "..", "..", "examples", "top"))


def ex(name):
    return os.path.join(EX, name)


def test_module_exports_version():
    assert isinstance(vyges_loom.__version__, str)
    assert vyges_loom.__version__  # non-empty


def test_load_dispatches_by_extension():
    d = vyges_loom.Design()
    assert d.load(ex("top.v")) == "netlist"
    assert d.load(ex("cells.lib")) == "liberty"
    assert d.load(ex("top.sdc")) == "sdc"
    assert d.load(ex("top.spef")) == "spef"


def test_loads_full_design_and_queries():
    d = vyges_loom.Design()
    for f in ("top.v", "cells.lib", "top.sdc", "top.spef"):
        d.load(ex(f))

    # netlist: the three-inverter chain
    nl = d.netlist
    assert nl is not None
    assert nl.module == "top"
    assert nl.inputs == ["a"]
    assert nl.outputs == ["y"]
    assert len(nl) == 3
    assert len(nl.instances) == 3
    assert {i.cell for i in nl.instances} == {"INV"}
    assert {i.name for i in nl.instances} == {"g1", "g2", "g3"}

    # every instance has an A input and a Y output connection
    g1 = next(i for i in nl.instances if i.name == "g1")
    conns = dict(g1.connections)
    assert conns["A"] == "a"

    # liberty / sdc / spef presence
    assert d.lib_cell_count >= 1
    assert d.liberty_count == 1
    assert d.has_sdc
    assert d.has_spef
    assert not d.has_def

    # cross-step provenance recorded in order
    assert [s.kind for s in d.steps] == ["netlist", "liberty", "sdc", "spef"]


def test_json_and_summary_contracts():
    d = vyges_loom.Design()
    d.load(ex("top.v"))
    d.load(ex("cells.lib"))

    # to_json() is valid JSON and carries the module name
    obj = json.loads(d.to_json())
    assert obj["netlist"]["module"] == "top"
    assert obj["netlist"]["instances"] == 3

    assert "netlist" in d.summary()
    assert repr(d).startswith("Design(")


def test_unknown_extension_raises():
    d = vyges_loom.Design()
    with pytest.raises(ValueError):
        d.load("design.gds")


def test_missing_netlist_is_none():
    d = vyges_loom.Design()
    assert d.netlist is None
    assert d.lib_cell_count == 0
