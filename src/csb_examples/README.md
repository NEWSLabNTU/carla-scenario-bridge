# csb_examples: starter scenarios

Scenarios that run on the carla-scenario-bridge pipeline as installed, no clone needed:

```bash
SCENARIOS=$(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios
export CARLA_MAPS=/path/to/maps          # holds Town01/, Town02/ (lanelet2_map.osm, pointcloud_map.pcd, ...)
ros2 run csb_launch run_suite $SCENARIOS/basic --output ~/csb_results/basic
```

Every file names its map as `$(env CARLA_MAPS)/<Town>`, so `CARLA_MAPS` must point at the
directory of converted towns (the TUM carla-autoware-bridge pack; see the user guide). Run a
single one with `play_launch launch --enforce-rules off --parser python --web-addr
127.0.0.1:8081 csb_launch scenario.launch.xml scenario:=$SCENARIOS/basic/town01_ego_drive.xosc
output_directory:=<dir>`. All of them assume the simulation side (CARLA,
`simulation.launch.xml`) and the ego's vehicle side are up, as for any scenario; every one
with an ego drives in Town01, so start the vehicle side with `map_path:=$CARLA_MAPS/Town01`.

Times are wall-clock on the development host (32 cores, one RTX GPU, CARLA 0.9.16 at Epic
quality), ego stack already running, measured by `run_suite` on 2026-10-10. They include
SSv2 start-up and the ego's engage, which is most of a short scenario.

## basic/ -- this project's own scenarios

| Scenario | Town | What it checks | Time |
| --- | --- | --- | --- |
| `town01_ego_drive.xosc` | Town01 | The ego, routed by `AcquirePositionAction`, engages and drives to its goal (westbound, x 190.8 -> 120). The smoke test. | 45 s |
| `town01_engage_state.xosc` | Town01 | Same drive, judged on Autoware's own state: `DRIVING` seen, then `ARRIVED_GOAL`. | 47 s |
| `town01_pedestrian.xosc` | Town01 | The ego passes a misc object (street barriers) and stops for a pedestrian crossing its lane, then resumes. | 128 s |
| `town01_rear_contact.xosc` | Town01 | A blind NPC runs into the ego from behind; SSv2's `CollisionCondition` must see it (compare with csb's CARLA collision summary). | 35 s |
| `town01_simulator_autopilot.xosc` | Town01 | Three drivers in one scenario: Autoware ego, an SSv2-driven NPC, and a CARLA Traffic Manager NPC (`simulator_autopilot`). | 50 s |
| `town01_traffic_light.xosc` | Town01 | The ego stops at an SSv2-commanded red light, then proceeds on green. | 187 s |
| `town02_episode_change.xosc` | Town02 | No entities: makes csb load Town02 (world reload, new episode). Succeeds after 5 s of simulation time; the next Town01 scenario loads Town01 back. | 71 s |

`run_suite` runs a directory in sorted order, so the Town02 scenario runs last in `basic/`.

## multi_av/ -- needs a second vehicle side

| Scenario | Town | What it checks |
| --- | --- | --- |
| `town01_two_av.xosc` | Town01 | The ego and `bg_av_1`, a second Autoware-driven vehicle (controller `agent`) 90 m ahead in the same lane; both reach their goals without colliding. Needs a vehicle side registered as `bg_av_1` with the agent relay (in this repo: `just bg-av`, or `just two-av` for the whole thing). |

## awf/ -- AWF ODD use cases

From [autowarefoundation/odd-usecase-scenario](https://github.com/autowarefoundation/odd-usecase-scenario),
`scenarios/` at commit `b258e781fdd487b866c3ecc2912e41239b47fef2`, Apache License 2.0;
attribution and the list of changes are in each file's header comment. They are TIER IV
YAML scenarios with `ScenarioModifiers`: SSv2 expands each into one run per combination,
and each combination is one testcase in the verdict.

| Scenario | Variants | What it checks |
| --- | --- | --- |
| `UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml` | 27 (ego speed 5.6/8.3/9.7 m/s x NPC speed 2.8/4.2/5.6 m/s x NPC target 1.4/1.9/2.8 m/s) | A preceding vehicle spawns 30 m ahead and decelerates at 1.5 m/s^2 to a lower speed; the ego must match it, without colliding and without braking harder than 1.5 m/s^2. |
| `UC-AEB-001-0001_Standing-pedestrian.yaml` | 9 (ego speed 5.6/8.3/9.7 m/s x pedestrian distance 15/30/50 m) | A pedestrian appears standing in the lane ahead; the ego must stop short of it, braking no harder than 6 m/s^2. |

### Map adaptation

The use cases are written for AWF's own `scenarios/maps/Town01.osm` (124 lanelets, ids from
1000 up). Our Town01 (`$CARLA_MAPS/Town01/lanelet2_map.osm`, the TUM conversion, 300
lanelets) shares **no lanelet ids** with it. Both scenarios use one lanelet only:

| AWF lanelet | Our lanelet | Geometry |
| --- | --- | --- |
| `1071` | `5230` | Same CARLA lane (OpenDRIVE road 4, lane -1; eastbound in ROS, y ~ -133.4). Both start at x = 101.42 and are 224.22 m long, so every `s` carries over unchanged. Projected through each map's own projector (what SSv2 and Autoware use), the bounds agree to 0.01 m: y = -131.41 / -135.41 at x = 101.42, -131.52 / -135.51 at x = 325.64, centred on CARLA's lane centre (CARLA y = 133.42 .. 133.50). Beware: our pack's `local_x`/`local_y` tags are stale (0.9 m off in y); compare by lat/lon. |

Per file:

- `UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml`: `laneId 1071 -> 5230` in
  all 6 `LanePosition`s (ego start s 5, ego goal s 200, NPC spawn and its trigger s 100,
  end conditions s 170); `s` and `offset` unchanged.
- `UC-AEB-001-0001_Standing-pedestrian.yaml`: `laneId 1071 -> 5230` in all 7
  `LanePosition`s (ego start s 5 and goal s 130, pedestrian s 100); `s` and `offset`
  unchanged.
- Both: `LogicFile` `maps/Town01.osm -> $(env CARLA_MAPS)/Town01`; `SceneGraphFile`
  (`maps/dummy.pcd`) removed, as in this project's other scenarios.
- Both: the ego's vehicle placeholders, which the AWF evaluator fills with its own vehicle's,
  set to this project's ego (`acb_vehicle`, CARLA's `vehicle.tesla.model3`):
  `__ego_dimensions_length__/width__/height__` 4 / 2 / 1.5 -> 4.792 / 2.163 / 1.488,
  `__ego_center_x__` 0 -> 1.532 (box centre ahead of the rear axle, the ego's origin),
  `__ego_center_z__` 0 -> 0.744. The bridge places the CARLA car by the declared box centre
  (`origin_offset_of` in `coordinator.rs`), so the upstream box, centred on the rear axle,
  spawned the car 1.39 m behind the pose SSv2 initialized Autoware's localization at; the
  concealer then refused to route ("did not reach it within 10000 ms ... tolerance 1 m") and
  all 9 AEB variants failed before engage in the first run.

### Results on the CARLA pipeline

Run 2026-10-10 with `run_suite` over `awf/`, CARLA 0.9.16 Town01, the ego stack on defaults:

| Scenario | Result | Time |
| --- | --- | --- |
| `UC-ACC-001-0001` | 27/27 variants passed | 2098 s (~78 s per variant) |
| `UC-AEB-001-0001` | 9/9 variants passed | 663 s (~74 s per variant) |

Two earlier runs failed, both on pipeline faults, both fixed:

1. All 9 AEB variants failed before engage on the ego box placeholder (see "Map adaptation"
   above); fixed in the scenario files.
2. With the box fixed, 8/9 AEB variants passed; variant 5 (ego 8.333 m/s, pedestrian 50 m)
   failed `AccelerationCondition < -6`: the ego read -6.61 m/s^2. The bridge's warnings put
   every such reading at **speed 0.00** -- the instant the car came to rest. Autoware had
   commanded at most -3.4 m/s^2 (its `stopped_acc`); the spike was the vehicle interface
   locking CARLA's handbrake as soon as Autoware's commanded velocity reached zero, with the
   car still rolling at ~0.8 m/s, which stopped it within two 50 ms ticks. Every variant
   produced one (-6.0 .. -6.6 m/s^2); most landed just after the verdict. acb now engages the
   standstill handbrake only below 0.1 m/s (`hold_with_handbrake` in acb's
   `vehicle_control.rs`); after it no ego deceleration beyond the bridge's -5 m/s^2 warning
   bound was logged in 36 variants.
