"""SSv2's protos in swb_world_protocol are the SSv2 fork's, byte for byte (roadmap 051).

The two sides of SSv2's wire are built from different files: the fork's runner from its own,
the bridge (through swb_core) from swb_world_protocol's copy. A field added to one and not
the other (roadmap 015's `simulation_time_ns`) decodes as silence, not as an error. This
repository holds both, so the check lives here.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OURS = ROOT / "src/swb_world_protocol/proto"
FORK = ROOT / "src/scenario_simulator_v2/simulation/simulation_interface/proto"
SSV2 = ["simulation_api_schema", "geometry_msgs", "traffic_simulator_msgs",
        "builtin_interfaces", "autoware_control_msgs", "autoware_vehicle_msgs",
        "rosgraph_msgs", "std_msgs"]


def test_ssv2_protos_are_the_forks():
    assert FORK.is_dir(), f"{FORK} missing: git submodule update --init --recursive"
    for name in SSV2:
        ours, fork = OURS / f"{name}.proto", FORK / f"{name}.proto"
        assert ours.read_bytes() == fork.read_bytes(), f"{ours} differs from {fork}"
