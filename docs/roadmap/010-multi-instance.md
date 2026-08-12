# Phase 010: Multi-Instance and Background AVs

Several Autoware stacks driving real vehicles in one CARLA world.

**Source**: gaps 9-10 from
[multi-instance-architecture.md](../design/multi-instance-architecture.md), plus audit
findings D1 and D2.
**Depends on**: [007](007-repeatable-runs.md) for lifecycle, [009](009-map-and-traffic-lights.md)
for map loading.

## Problem

Two prerequisites block the feature before any fan-out work starts.

`config/bridge_config.yaml` is **never read**. There is no config loading anywhere in the
crate: `Coordinator::new(world)` takes no config, `serde` and `serde_yaml` are declared
dependencies and unused, and all real configuration arrives through environment variables in
`main.rs`. The documented `blueprint_map` mechanism does not exist — `spawn_vehicle_entity`
uses the raw `asset_key` with a hardcoded `vehicle.tesla.model3` fallback.

The ego's `role_name` is hardcoded:

```rust
if is_ego {
    match builder.set_attribute("role_name", "hero") { ... }
}
```

`acb_bridge` finds its vehicle by `role_name`, so with one fixed name there is exactly one
bridge, one Autoware, one domain. Background AVs need distinct names before anything else can
work.

The scale is bounded by upstream: SSv2 throws `"Multiple egos in the simulation are
unsupported yet."` and its concealer forks Autoware into SSv2's own `ROS_DOMAIN_ID`. So one
SSv2 drives one scenario ego, and additional Autoware instances are *background AVs* outside
SSv2's model — invisible to its conditions. See the design doc for why.

## Work Items

### Config loading (D1)

- [x] Load `bridge_config.yaml` at startup; make `serde`/`serde_yaml` earn their place
- [x] Precedence between file and environment is defined and documented
- [x] Wire `blueprint_map` into spawn so the documented mechanism works
- [x] Missing or malformed config fails at startup with a clear message, not at first use
- [x] Delete any config key that nothing reads

**Precedence: environment > file > default.** The environment is the more specific source —
launch files set `CARLA_HOST` / `CARLA_PORT` / `SSV2_PORT` per run, and a checked-in config
must not silently win over what a launch explicitly asked for.

A *missing* file is fine, since every key has a default. A *malformed* one is fatal: falling
back to defaults would run the scenario with settings the operator believes they changed.
`timeout_ms` was dropped — nothing read it.

### Role names (D2)

- [x] Ego `role_name` is configurable, defaulting to `hero` for compatibility
- [x] Background AVs get distinct role names from config
- [x] Names are unique; a collision is rejected at startup, not discovered at spawn

`role_name` is no longer a property of the spawn kind — it is passed in, so the ego takes its
name from config and each background AV takes its own. Duplicates (including one colliding
with the ego) are rejected by `BridgeConfig::validate`, because `acb_bridge` finds its vehicle
by `role_name` and a duplicate means two bridges racing for one vehicle.

### Background AV spawning (gap 9)

- [x] `background_avs` config section: role name, blueprint, spawn pose, goal pose, domain
- [x] Spawn them at `Initialize`, after map load and before the ego
- [x] They are tracked for teardown like every other spawned actor (invariant 2, phase 007)
- [x] They are **not** registered with SSv2 — no `EntityManager` entry that would reach
      `UpdateEntityStatus`
- [x] Their pose authority is CARLA PhysX, never teleport (invariant 5)
- [x] Empty list behaves exactly like today's single-ego run

`SpawnKind::entity_type` returns `Option<EntityType>` and `None` for a background AV. That is
the mechanism keeping it out of SSv2's view: registering it would put it into
`UpdateEntityStatus`, and SSv2 would start teleporting a vehicle Autoware is already driving —
two pose authorities on one actor.

Spawning happens after the map (a reload destroys actors) and before the ego, so the
background Autoware instances can be finding their vehicles while SSv2 is still setting up. A
background AV that fails to spawn is reported loudly but does not abort the scenario; the ego
can still run, and killing a whole scenario because a secondary vehicle would not fit is the
wrong trade.

### Per-domain launch

- [x] Launch file bringing up Autoware + `acb_bridge` for one background AV in a given domain
- [x] `publish_clock:=true` in background domains, `false` in the ego's — already supported
- [x] A pilot per background domain to set route and engage, since no concealer is present
- [x] Document the startup order across domains

`csb_launch/launch/background_av.launch.xml` brings up Autoware, `acb_bridge` and the pilot
for one AV.

The pilot is wired but unverified. `acb_pilot`'s `auto_drive` node now launches per domain:
it waits for the AD API services and GNSS-driven localization, sets the route, engages, and
exits on arrival — the concealer's job, minus the concealer. It runs unless
`use_pilot:=false`.

The goal does **not** come from `bridge_config.yaml`. The pilot's only input is a
`poses_file` ROS parameter — a YAML with `goal_pose: {x, y, z, qx, qy, qz, qw}` — while the
config records `goal_pose: {x, y, z, yaw}` inline. A file path versus an inline pose, a
quaternion versus a yaw: aligning them would mean either changing the pilot (a read-only
submodule) or generating pose files at launch time. So the launch takes a
`goal_poses_file` argument, and the config's `goal_pose` stays what it always was to the
bridge — an informational record of intent, read by nobody.

What still needs a live stack: that the pilot's engage actually succeeds in a background
domain (its timing waits were tuned in single-domain runs), and that failure is loud. The
failure mode is bounded, though — an empty `goal_poses_file` is a fatal log and a nonzero
exit, not a vehicle that silently never moves.

### Concealer web-addr (gap 10)

- [x] Parameterise the hardcoded `--web-addr 0.0.0.0:8082` in our SSv2 `launch.hpp` patch
- [x] Confirm no port collision when several play_launch-managed stacks run on one host
      (2026-08-10: `PLAY_LAUNCH_WEB_ADDR=0.0.0.0:8083` for the D1 stack; no collision.
      The REAL port collision was DDS: see the DomainGain note below)
- [x] Keep the patch minimal — it is rebased onto upstream regularly

Now reads `PLAY_LAUNCH_WEB_ADDR`, falling back to `0.0.0.0:8082`. Four added lines, so the
rebase burden is unchanged. Confirming no collision needs two stacks actually running.

### Documentation

- [x] Record that background AVs are invisible to SSv2 conditions, with the misc-object
      registration escape hatch described in the design doc
- [x] Worked two-domain example

Both live in [multi-instance-architecture.md](../design/multi-instance-architecture.md), and
the invisibility warning is repeated in `bridge_config.yaml` where an operator adding a
background AV will actually read it.

### Tests

- [x] Unit: config parsing, including background AV lists and duplicate-name rejection
- [x] Unit: `blueprint_map` resolution
- [x] Unit: a background AV is not an SSv2 entity, and is physics-driven
- [x] Integration: configured background AVs appear with correct role names (live:
      `bg_av_1` spawned before the ego, found by its own acb_bridge in domain 1)
- [x] Integration: background AVs are absent from SSv2 entity status (live: ego
      scenario ran to `Passed` with `bg_av_1` in-world; SSv2 never saw it)
- [ ] Integration: all background AVs are destroyed at teardown

## Acceptance Criteria

- [x] `bridge_config.yaml` is read, and every key in it does something
- [x] Ego role name is configurable and defaults to `hero`
- [x] A two-domain run works: scenario ego plus one background AV, both localize and drive
      — **verified 2026-08-10** on newslab-server243: both stacks coexist and discover
      cleanly, `bg_av_1` spawns, its acb attaches and GNSS-initializes localization at the
      spawn pose, its pilot **sets a route and engages** (`Route set successfully`, then
      `Driving... route_state=2, op_mode=2`) while the ego drives its scenario to
      `Passed`. Five consecutive ego passes on this host.
      **Arrival too, 2026-08-12**, on `scenarios/town01_two_av.xosc`:

      ```
      Localization: INITIALIZED at (139.95, -55.48)   <- its own spawn pose
      Route set successfully (attempt 1)
      Autonomous mode engaged
      ARRIVED at goal after 18.8s
      ```

      with the ego passing its own scenario in the same run. Two Autoware stacks driving
      two real vehicles in one world, both reaching their goals, SSv2 scoring one of them
      and never seeing the other.

      This needed nothing but sim time. The pilot's cold start is ~62 s from spawn to
      arrival — 16 s to localize, 17 s settling, 10 s to engage, 19 s to drive its 30 m —
      and the world only ticks while a scenario runs, so `town01_ego_drive.xosc` at ~50 s
      used to cut it off seconds after engage. `town01_two_av.xosc` runs the ego 212 m
      instead of 71 m (x 320 -> 108) and raises the scenario timeout to 300 s. That whole
      stretch is one lanelet, 6583, which carries **no regulatory element** — worth
      keeping when extending a route, because csb freezes the traffic lights and a red one
      on the way would strand the ego for the rest of the run.

      Budget rule for any new multi-AV scenario: the ego must stay in motion for at least
      ~65 s of sim time, or the background AV will not finish.
      Domains changed since this was written: the ego and SSv2 share **domain 1** and
      background AVs start at **2**, leaving 0 empty so no unconfigured ROS process on
      the host can join a run.

      Four prerequisites were fixed on the way and matter to anyone rerunning this:
      1. **CycloneDDS `DomainGain 1000`** (config/cyclonedds-localhost.xml): the default
         (250) puts domain 1's unicast port base inside domain 0's range once
         `MaxAutoParticipantIndex` is 300 — launching the D1 stack then breaks NEW
         domain-0 participants with "Failed to find a free participant index".
      2. **On-lanelet, on-graph, correctly-directed pilot goal**
         (scenarios/bg_av_1_poses.yaml). "The planned route is empty" has three distinct
         causes and one message. The design doc's example goal (x=40) is past the end of
         its lanelet. The obvious replacement lane (y=-53.7) carries **no subtype tag** —
         65 of this map's 300 lanelets do not — and Autoware routes over `subtype=road`
         only. And the lane that is tagged (16946) runs **east to west**, so a vehicle
         facing east has its goal behind it. Working pair: spawn (140, -55.5) yaw 180,
         goal (110, -55.5) yaw 180. Check candidates with lanelet2's own `RoutingGraph`,
         not by eye off the OSM: left and right boundaries are not stored in a consistent
         order, so a hand-built centerline can come out reversed.
      3. **play_launch's compound-parameter fix** (`f78745d`): without it
         `autoware_pose_initializer_node` and `shape_estimation` abort at startup, so the
         background stack never localizes at all.
      4. **acb noticing a despawn** (acb `ab5dc67`): `Actor::IsAlive()` is client-side and
         never went false for a vehicle csb destroyed, so on the *second* scenario run a
         background AV's bridge kept publishing sensor topics with no CARLA actors behind
         them and its Autoware was fed nothing. Both bridges now re-attach within seconds
         of the next run's first tick. Before this, a second run needed the whole stack
         restarted; it no longer does.
- [x] Each domain has exactly one `/clock` publisher (verified in D0 after the
      double-acb_bridge fix, csb `b974590`; D1 uses `publish_clock:=true` by design)
- [ ] The background AV is perceived by the ego's Autoware through its sensors
- [x] SSv2 neither sees nor controls the background AV
- [x] An empty `background_avs` list is byte-for-byte the current behaviour
- [x] Teardown leaves no background AV behind (2026-08-10: after each of five runs the
      CARLA world holds no vehicles and no sensors)
- [x] `just test` passes

One criterion is left: whether the ego's Autoware *perceives* the background AV through
its sensors. That needs the two to be near each other during a run — today the bg AV
drives y=-55.5 while the ego drives y=-129.8, two parallel streets, so the question has
not been posed yet. Posing it means putting the background AV on the ego's street ahead
of it: the ego's lanelet 6583 runs x 325.6 -> 101.4 westbound, and it now covers
x 320 -> 108 of that, so there is room for a slower vehicle in front. Expect the ego to
*react* as well as perceive — which is worth watching, since SSv2 scores the ego's
arrival but cannot see what it is avoiding.
