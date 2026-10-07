#!/usr/bin/env python3
"""Generate the NPC-only Town01 frame-budget benchmarks (roadmap 016 step 4).

Each scenario declares N puppeteered NPC vehicles and no ego, so it needs no Autoware: SSv2
drives every NPC along its lane at a fixed speed and csb applies each pose with
set_transform, which is the per-entity path whose cost grows with N. The run lasts 60 s of
simulation time and succeeds on a SimulationTimeCondition; csb's "Frame budget, whole run"
line at the next Initialize (or at shutdown) is the measurement.

NPCs start on straight lanelets away from junctions, spread round-robin across lanelets so no
road carries them all, at least SPACING m apart within a lanelet. Positions are LanePositions
(lanelet ID + s), which SSv2 resolves itself, so no pose has to be hand-matched to a lane.

Usage:
    scripts/gen_npc_benchmark.py                 # writes scenarios/bench/town01_npc_{10,20,50}.xosc
    scripts/gen_npc_benchmark.py --counts 5 100 --out /tmp/bench
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAP = ROOT / "data/carla-autoware-bridge/Town01/lanelet2_map.osm"
DEFAULT_OUT = ROOT / "scenarios/bench"

DURATION_S = 60
SPEED_MPS = 8.0
# Start positions: margin from either end of a lanelet, and gap between NPCs on one lanelet.
MARGIN = 10.0
SPACING = 25.0
# A lanelet counts as straight road (not a junction connector) when its heading changes by
# less than this between its first and last centerline segment, and it is at least this long.
MAX_TURN_DEG = 10.0
MIN_LENGTH = 2 * MARGIN + 5.0


def load_lanelets(path):
    """Return [(lanelet_id, length_m)] for straight road lanelets, sorted by id."""
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.iter("node"):
        tags = {t.get("k"): t.get("v") for t in n.iter("tag")}
        if "local_x" in tags:
            nodes[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.iter("nd")] for w in root.iter("way")}

    out = []
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        members = {m.get("role"): m.get("ref") for m in rel.iter("member") if m.get("type") == "way"}
        if "left" not in members or "right" not in members:
            continue
        left = [nodes[r] for r in ways[members["left"]] if r in nodes]
        right = [nodes[r] for r in ways[members["right"]] if r in nodes]
        center = centerline(left, right)
        if len(center) < 2:
            continue
        length = sum(math.dist(a, b) for a, b in zip(center, center[1:]))
        if length < MIN_LENGTH or turn_deg(center) > MAX_TURN_DEG:
            continue
        out.append((int(rel.get("id")), length))
    return sorted(out)


def centerline(left, right):
    """Midpoints of the bounds resampled to the shorter one's point count."""
    n = min(len(left), len(right))
    if n < 2:
        return []
    pick = lambda pts, i: pts[round(i * (len(pts) - 1) / (n - 1))]
    return [
        ((pick(left, i)[0] + pick(right, i)[0]) / 2, (pick(left, i)[1] + pick(right, i)[1]) / 2)
        for i in range(n)
    ]


def turn_deg(pts):
    h0 = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    h1 = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
    d = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
    return abs(math.degrees(d))


def start_positions(lanelets, count):
    """`count` (lanelet_id, s) slots, round-robin over lanelets so NPCs spread out."""
    slots = []
    for lanelet_id, length in lanelets:
        k = int((length - 2 * MARGIN) // SPACING) + 1
        slots.append([(lanelet_id, MARGIN + i * SPACING) for i in range(k)])
    picked = []
    depth = 0
    while len(picked) < count:
        layer = [s[depth] for s in slots if depth < len(s)]
        if not layer:
            sys.exit(f"Town01 has room for only {len(picked)} NPCs at {SPACING} m spacing")
        picked.extend(layer[: count - len(picked)])
        depth += 1
    return picked


VEHICLE = """		<ScenarioObject name="{name}">
			<Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
				<BoundingBox>
					<Center x="1.5" y="0.0" z="0.9" />
					<Dimensions width="2.1" height="1.8" length="4.5" />
				</BoundingBox>
				<Performance maxSpeed="20" maxAcceleration="5.0" maxDeceleration="8.0" />
				<Axles>
					<FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3" />
					<RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8" positionX="0.0" positionZ="0.3" />
				</Axles>
				<Properties />
			</Vehicle>
			<ObjectController>
				<Controller name="">
					<Properties />
				</Controller>
			</ObjectController>
		</ScenarioObject>
"""

INIT = """				<Private entityRef="{name}">
					<PrivateAction>
						<TeleportAction>
							<Position>
								<LanePosition roadId="" laneId="{lanelet}" s="{s:.1f}" offset="0.0">
									<Orientation type="relative" h="0" p="0" r="0" />
								</LanePosition>
							</Position>
						</TeleportAction>
					</PrivateAction>
					<PrivateAction>
						<LongitudinalAction>
							<SpeedAction>
								<SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" />
								<SpeedActionTarget>
									<AbsoluteTargetSpeed value="{speed}" />
								</SpeedActionTarget>
							</SpeedAction>
						</LongitudinalAction>
					</PrivateAction>
				</Private>
"""

TEMPLATE = """<?xml version="1.0"?>
<!--
	Town01, {n} puppeteered NPC vehicles and no ego: a frame-budget benchmark (roadmap 016
	step 4). Generated by scripts/gen_npc_benchmark.py; regenerate rather than edit. Each NPC
	starts on a straight lanelet and SSv2 drives it along its lane at {speed} m/s; csb applies
	every pose with set_transform. Succeeds after {duration} s of simulation time. The
	measurement is csb's "Frame budget, whole run" line, logged at the next Initialize.
-->
<OpenSCENARIO>
	<FileHeader author="carla-scenario-bridge" date="2026-10-07T00:00:00+00:00" description="Town01 frame-budget benchmark, {n} NPC vehicles, no ego" revMajor="1" revMinor="2" />
	<ParameterDeclarations />
	<CatalogLocations />
	<RoadNetwork>
		<LogicFile filepath="$(find-pkg-share csb_launch)/data/carla-autoware-bridge/Town01" />
	</RoadNetwork>
	<Entities>
{entities}	</Entities>
	<Storyboard>
		<Init>
			<Actions>
{init}			</Actions>
		</Init>
		<Story name="town01_npc_{n}">
			<Act name="bench_act">
				<ManeuverGroup maximumExecutionCount="1" name="end_group">
					<Actors selectTriggeringEntities="false" />
					<Maneuver name="end_conditions">
						<Event name="success" priority="parallel">
							<Action name="exit_success">
								<UserDefinedAction>
									<CustomCommandAction type="exitSuccess" />
								</UserDefinedAction>
							</Action>
							<StartTrigger>
								<ConditionGroup>
									<Condition name="duration" delay="0" conditionEdge="none">
										<ByValueCondition>
											<SimulationTimeCondition value="{duration}" rule="greaterThan" />
										</ByValueCondition>
									</Condition>
								</ConditionGroup>
							</StartTrigger>
						</Event>
					</Maneuver>
				</ManeuverGroup>
				<StartTrigger>
					<ConditionGroup>
						<Condition name="start" delay="0" conditionEdge="none">
							<ByValueCondition>
								<SimulationTimeCondition value="0" rule="greaterThan" />
							</ByValueCondition>
						</Condition>
					</ConditionGroup>
				</StartTrigger>
			</Act>
		</Story>
		<StopTrigger />
	</Storyboard>
</OpenSCENARIO>
"""


def scenario(n, positions):
    names = [f"npc_{i:02d}" for i in range(n)]
    entities = "".join(VEHICLE.format(name=name) for name in names)
    init = "".join(
        INIT.format(name=name, lanelet=lanelet, s=s, speed=SPEED_MPS)
        for name, (lanelet, s) in zip(names, positions)
    )
    return TEMPLATE.format(
        n=n, entities=entities, init=init, speed=SPEED_MPS, duration=DURATION_S
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--counts", type=int, nargs="+", default=[10, 20, 50])
    ap.add_argument("--map", type=Path, default=MAP)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    lanelets = load_lanelets(args.map)
    args.out.mkdir(parents=True, exist_ok=True)
    for n in args.counts:
        path = args.out / f"town01_npc_{n}.xosc"
        path.write_text(scenario(n, start_positions(lanelets, n)))
        print(f"{path} ({n} NPCs, {len(lanelets)} candidate lanelets)")


if __name__ == "__main__":
    main()
