# Phase 008: Entity Fidelity

Pedestrians, misc objects, and NPCs that behave the way SSv2 believes they do.

**Source**: workflow audit 2026-07-28, findings A1, A2, C1, C3.
**Supersedes**: [003-npc-pedestrian-support.md](003-npc-pedestrian-support.md), which
predates the authority model.
**Depends on**: [006](006-honest-failures.md) for honest rejection, [007](007-repeatable-runs.md)
for repeatable testing.

## Problem

Two entity types are unimplemented (phase 006 makes that visible), and the one type that *is*
implemented is puppeteered incorrectly.

Invariant 5 says a vehicle's pose has exactly one authority: SSv2 teleport, or CARLA PhysX.
For scenario NPCs it is SSv2 — `update_entity_status` calls `set_transform()` each frame. But
the actors are spawned with CARLA physics **enabled**, so PhysX also acts on them between
ticks: gravity pulls, collisions shove, suspension settles. Two authorities, one actor.

The AWSIM pattern this design follows makes puppeteered actors kinematic. Without
`set_simulate_physics(false)` the NPC's actual pose drifts from the commanded pose, and the
pose reported back to SSv2 is the commanded one anyway — so SSv2 cannot see the drift.

Ego readback also reports three constants:

```rust
current_action: String::new(),
linear_jerk: 0.0,
angular: Some(geometry_msgs::Vector3 { x: 0.0, y: 0.0, z: 0.0 }),
```

Any SSv2 condition reading action state or jerk is reading a constant, not the simulation.

## Work Items

### NPC physics (C1)

- [x] Disable physics on scenario NPCs at spawn, so `set_transform` is the only authority
- [x] Decide and document the pedestrian equivalent — CARLA walkers differ from vehicles
- [x] Verify the commanded pose and CARLA's reported pose agree within tolerance after a tick
- [x] Document the consequence in the design doc: NPC-to-NPC and NPC-to-ego collisions are
      not simulated by CARLA, matching AWSIM; SSv2 owns collision detection

Pose authority is now a property of the entity kind rather than scattered `is_ego` checks.
`SpawnKind::physics_driven()` is true only for the ego; everything else gets
`set_simulate_physics(false)` at spawn. Walkers need no special handling — SSv2 sends a pose
every frame, so they are teleported exactly like NPC vehicles.

Verified live on 2026-07-30 — see the bottom of this document.

### Pedestrians (A1)

- [x] `spawn_pedestrian_entity` spawns a CARLA walker at the converted pose
- [x] Register in `EntityManager` as `EntityType::Pedestrian`
- [x] `update_entity_status` drives walkers by pose, consistent with the NPC path
- [x] Walker despawn works through the existing `despawn_entity`
- [x] Decide whether an AI walker controller is used at all

**No AI walker controller.** SSv2's behaviour plugins compute the walk and send a pose every
frame; a controller would be a second authority over the same actor. `update_entity_status`
and `despawn_entity` are kind-agnostic and needed no change.

### Misc objects (A2)

- [x] `spawn_misc_object_entity` spawns a static prop
- [x] Register as `EntityType::MiscObject`
- [x] Map SSv2 asset keys to CARLA prop blueprints, with a documented fallback
- [x] Static objects are not teleported per frame unless SSv2 moves them

### Ego readback fidelity (C3)

- [x] Populate `current_action` from real state, or document why empty is correct
- [x] Compute `linear_jerk` from acceleration history
- [x] Populate angular acceleration, or document the omission
- [x] Whatever is left constant is recorded as a known limitation, not left silent

`current_action` **stays empty, deliberately**: it names the behaviour-plugin action driving
an entity, SSv2 owns that for the NPCs it puppeteers (their status is echoed back untouched),
and the ego has no SSv2 behaviour plugin — it is driven by Autoware, whose action has no
equivalent in this field.

`linear_jerk` is now real. CARLA reports acceleration but not its derivative, so acceleration
is projected onto the heading to get a signed longitudinal scalar and differenced across
frames. The projection matters: the vector magnitude would report braking and accelerating
identically. A non-positive step time yields 0.0 rather than an infinity SSv2 would evaluate
conditions against.

**Angular acceleration remains zero — a known limitation.** CARLA exposes angular velocity but
not its derivative, and differencing it at a 20 Hz step is too noisy to report as truth.

### Blueprint mapping

- [x] SSv2 asset key → CARLA blueprint resolution for all four entity types
- [x] Unresolvable keys log a warning naming the key and the fallback used
- [x] Depends on config loading from [010](010-multi-instance.md); mapping lives in code
      with a pointer to that phase

Fallbacks are per kind, so a pedestrian cannot fall back to a car nor a prop to a walker —
covered by a test. SSv2 asset keys are not CARLA blueprint names in general; where they
coincide they pass through, otherwise the kind default applies with a warning naming both.

### Tests

- [x] Unit: only the ego is physics-driven, and only the ego carries a `role_name`
- [x] Unit: each kind maps to its `EntityType` and to a same-family fallback blueprint
- [x] Unit: longitudinal projection keeps its sign; jerk is a rate and stays finite
- [x] Unit: asset key → blueprint resolution, including fallback and unknown-key paths —
      the decision is now `choose_blueprint`, split out of `resolve_blueprint_key` so it can
      be tested without a server: `actor_builder` takes `&mut World` and needs a live CARLA,
      which is what made the fallback and unknown-key paths untestable. Four cases covered —
      a key CARLA knows passes through unchanged, an unknown key falls back to the kind
      default, an unknown key with an unusable fallback is an error rather than a silent
      substitution, and an unprobed fallback is never assumed to work.
- [x] Integration: spawned pedestrian appears in CARLA and tracks its commanded pose
      — probe, re-verified 2026-08-08
- [x] Integration: spawned misc object appears and stays put — probe, 2026-08-08
- [x] Integration: NPC pose after a tick matches the commanded pose within tolerance
      — probe, 2026-08-08: commanded CARLA(125.0, 130.0), actual (125.0, 130.0)
- [x] Integration: all three types despawn cleanly, no leak (ties to 007) — probe's second
      `Initialize` left 0 vehicles, 0 walkers, 2026-08-08

`resolve_blueprint_key` probes CARLA, so it cannot be unit-tested without a live server —
same `Coordinator` constructibility problem noted in [007](007-repeatable-runs.md). The
kind→fallback mapping it depends on *is* tested.

## Acceptance Criteria

- [x] A pedestrian-crossing scenario runs with a pedestrian actually present in CARLA
- [x] Autoware's perception detects the pedestrian through its sensors
- [x] NPC vehicles follow their scripted trajectory without sinking, jittering or drifting
- [x] A misc object appears as an obstacle Autoware perceives
- [x] Every entity type spawns, updates and despawns without leaking
- [x] No handler introduced here returns success without acting
- [x] `just test` passes

The C1 physics fix — the one predicted from CARLA semantics rather than observed — **is
confirmed**; see the 2026-07-30 section below. The three perception criteria closed on
2026-08-16 against a live Autoware, below.

## Perception, verified live (2026-08-16)

`scenarios/town01_pedestrian.xosc` puts a pedestrian and two street barriers on the ego's
route through Town01, and `scripts/entity_perception_probe.py` reads CARLA's walkers and
props directly, matching each against the ego's tracked objects. Four runs, all
`failures="0"`.

**The pedestrian is detected, and the ego stops for it.** With the crossing timed to put
the walker in the ego's lane as it arrives, the ego halted at x=248.9 — 9 m short of the
crossing line — held for 53 s while the pedestrian crossed, then resumed the moment it
reached the far walkway and drove on to its goal:

```
t+ 31s ego( 253.2)  ped0001@(240,-131.8)  approaching
t+ 33s ego( 248.9)  ped0001@(240,-130.4) ego 12m HIT d=0.0 UNKNOWN
t+ 33s .. t+86s     ego held at 248.9 while the pedestrian crossed
t+ 88s ego( 247.8)  pedestrian clear at y=-126.1, ego moving again
t+119s ego( 139.7)  goal reached, exitSuccess
```

**The misc object is detected**: the walkway barrier at (200, -126.8), 3 m off the ego's
driving line, matched a tracked object to 0.0 m with the ego 4 m away.

Everything is classified **UNKNOWN**, never PEDESTRIAN. Detection is what the criteria
ask for and detection is what happens, but nothing here exercises classification, and the
camera-based leg that would is the same one [009](009-map-and-traffic-lights.md) has
parked for want of GPU headroom.

**Detection range is short.** `barrier_1` sits in the ego's own lane at x=120, which is
15 m past the ego's goal, so the ego never came closer than 20 m to it — and it was never
detected. The barrier that *was* detected came inside 11 m first. For a 1.2 m prop with
this sensor kit, plan on well under 20 m; a scenario that needs a prop seen has to route
the ego past it, not merely towards it.

**Teardown leaks nothing**, across all four entity types: after each run the world holds
0 vehicles, 0 sensors, 0 walkers and 0 non-map props.

Two traps in measuring any of this:

1. **Town01 contains ~150 `static.prop.mesh` actors** — the map's own scenery, part of
   the pointcloud map and subtracted from perception by design. A probe that reports
   every prop buries the two the scenario spawned in a wall of noise.
2. **For the first seconds after `Initialize`, perception still publishes the previous
   run's object list.** A leftover object landing on a new entity's coordinates scores as
   a detection at 200 m — which is how the probe first "detected" `barrier_1`. The probe
   now refuses any match beyond 80 m.

### FollowTrajectoryAction starts late

This map has **no crosswalk lanelets** — 169 road, 66 walkway, 65 untagged, and nothing
tagged crosswalk — so SSv2's usual pedestrian pattern (teleport onto a crosswalk lanelet,
give it a speed) has nothing to stand on. The crossing is a `FollowTrajectoryAction` over
world positions instead, which needs no lanelet.

Triggering it from a `ReachPositionCondition` on the ego did not work as written: on two
runs the walk began **~15 s after** the ego passed the trigger point, by which time the
ego was past the crossing and the pedestrian stepped into the lane behind it. The
scenario now starts the crossing on simulation time and holds the pedestrian in the lane
for 28 s, so the meeting does not depend on that lag being any particular value. The lag
itself is unexplained and worth a look if anything else starts depending on
action-trigger latency.


## Verified against live CARLA (2026-07-30)

`scripts/integration/ssv2_probe.py` drives the bridge over the real protocol and asserts
against the CARLA world. See that script's README.

**The C1 physics fix is confirmed.** An NPC commanded to ROS `(125.0, -130.0)` lands at
CARLA `(125.0, 130.0)` — exact to the printed precision. That validates both
`set_simulate_physics(false)` and the ROS↔CARLA Y-flip, the two things phase 008 could only
predict. Pedestrians and misc objects spawn and appear in the world.
