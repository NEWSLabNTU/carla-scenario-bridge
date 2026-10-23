#!/usr/bin/env python3
"""One simulation time in every repository (roadmap 051, decision 5): BASE_NS and the nanos
table are the same in swb_world_protocol (Rust and the Python shell), acb_bridge and aib.

    python3 -m pytest -q scripts/test_sim_time.py      (run by `just test`)
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUST = {
    "swb_core sim_time.rs": ROOT / "src/swb_world_protocol/swb_core/src/sim_time.rs",
    "acb clock.rs": ROOT / "src/autoware_carla_bridge/src/acb_bridge/src/clock.rs",
}
PYTHON = {
    "swb_world_server protocol.py":
        ROOT / "src/swb_world_protocol/swb_world_server/swb_world_server/protocol.py",
    "aib sim_time.py": ROOT / "src/autoware_isaac_bridge/src/aib/aib/sim_time.py",
}
TABLES = {
    "swb_core sim_time.rs": RUST["swb_core sim_time.rs"],
    "acb clock.rs": RUST["acb clock.rs"],
    "aib test_sim_time.py": ROOT / "src/autoware_isaac_bridge/src/aib/test/test_sim_time.py",
}


def base_ns(path):
    m = re.search(r"BASE_NS\s*(?::\s*i64)?\s*=\s*([0-9_]+)", path.read_text())
    assert m, f"{path}: no BASE_NS"
    return int(m.group(1).replace("_", ""))


def table(path):
    text = path.read_text()
    block = text[text.index("NANOS_CASES"):]
    block = block[block.index("= "):]  # past a Rust type's own brackets
    block = block[:block.index("]") + 1]
    return [(float(s), int(n.replace("_", "")))
            for s, n in re.findall(r"\(\s*(-?[0-9.e+-]+)\s*,\s*(-?[0-9_]+)\s*\)", block)]


def test_one_base_everywhere():
    values = {name: base_ns(p) for name, p in {**RUST, **PYTHON}.items()}
    assert set(values.values()) == {1_000_000_000_000}, values


def test_one_nanos_table_everywhere():
    tables = {name: table(p) for name, p in TABLES.items()}
    first = next(iter(tables.values()))
    assert len(first) >= 15
    for name, rows in tables.items():
        assert rows == first, name


def test_the_python_nanos_follow_the_table():
    import importlib.util

    for name, path in PYTHON.items():
        spec = importlib.util.spec_from_file_location(name.split()[0], path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for secs, ns in table(TABLES["swb_core sim_time.rs"]):
            assert module.nanos(secs) == ns, (name, secs)
