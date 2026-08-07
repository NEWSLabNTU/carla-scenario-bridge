# Phase 012: SSv2-Unmanaged Autoware

SSv2 stops launching and managing the ego's Autoware. Every Autoware stack — ego and
background alike — is brought up externally, and `csb_bridge` remains the only process
touching CARLA's world settings. SSv2 is reduced to what it is uniquely good at: parsing
scenarios, puppeteering NPCs, and judging conditions over ZMQ.

**Source**: replaces Design Principle 5 of [architecture.md](../design/architecture.md);
resolves the AutowareUniverse dual-publisher conflict tracked in
[multi-instance-architecture.md](../design/multi-instance-architecture.md#known-conflict-autowareuniverse-vehicle-status);
removes gap 10 (the concealer `launch.hpp` patch) entirely.
**Depends on**: the per-domain pilot from [010](010-multi-instance.md) — this phase makes the
ego just another consumer of it. Benefits [011](011-robustness.md) by deleting the
duplicate-publisher hazard instead of mitigating it.

## Problem

Design Principle 5 ("SSv2 Drives Autoware Lifecycle") was written at project scaffold and has
never been revisited. It predates the multi-instance authority model, and everything that
model had to work around traces back to it:

- **Dual vehicle-status publishers.** SSv2's `AutowareUniverse` node publishes
  `/vehicle/status/*` at 30 Hz from its internal bicycle model while `acb_bridge` publishes
  the same topics from CARLA PhysX. The two agree only while the models agree — and their
  divergence is exactly what a scenario is meant to detect.
  [ssv2-launch-configuration.md](../design/ssv2-launch-configuration.md#autowareuniverse-topic-conflict)
  accepts this as a "Phase 2 expedient"; it was never meant to be a resting state.
- **A fork patch we rebase forever.** The concealer hardcodes `--web-addr` for the Autoware
  it launches; our `launch.hpp` patch parameterises it. The patch exists only because SSv2
  forks Autoware at all.
- **Domain asymmetry.** `concealer::ros2_launch` is a plain `fork()` + `exec`, so the ego's
  Autoware always inherits SSv2's `ROS_DOMAIN_ID`. Background AVs get their own domains,
  their own `/clock` publisher, and a pilot; the ego gets SSv2's domain, a silenced
  `publish_clock:=false` special case, and the concealer. Two mechanisms for one job.
- **Ego lifecycle is invisible to us.** When the concealer fails to engage, the failure
  surfaces as a scenario timeout inside SSv2, not as anything this bridge can see, log, or
  retry.

The tick side of this refactor is already done: invariant 1 (only `csb_bridge` calls
`world.tick()` / `apply_settings()`) is implemented by the `FrameAction` state machine and
unit-tested. This phase is about the other half — Autoware lifecycle.

## What does not change

- The ZMQ protocol and all 14 handlers. SSv2 still spawns the ego via
  `SpawnVehicleEntity(is_ego=true)` and reads its pose back each frame.
- Pose authority: ego stays CARLA PhysX; NPCs stay teleported.
- Sensors, NPC puppeteering, traffic lights.

## Work Items

### Investigation first (spike)

The design hinges on what our pinned SSv2 (25.0.22) actually does when told not to launch
Autoware. Answer these against the real code before touching anything else:

- [ ] What does `launch_autoware:=false` do to `EgoEntity` / `FieldOperatorApplication`?
      Does the interpreter still spawn the ego over ZMQ, still tick, still evaluate
      conditions — or does it block waiting for an Autoware that never comes?
- [ ] Which scenario constructs silently die without a concealer: ego `AcquirePositionAction`
      (routing), engage-state conditions, `UserDefinedValueCondition` reading Autoware
      state? List them; they define what scenario authors lose.
- [ ] Does SSv2 still publish `/clock` in its own domain with no Autoware attached, and does
      anything still consume it there?

The spike's output is a short section in this file stating what is possible without forking
SSv2. If `launch_autoware:=false` cannot produce a spawning, ticking, condition-evaluating
run, this phase stops here and the fallback is a minimal upstream contribution to SSv2 —
not a deeper fork.

### Ego pilot

- [ ] The ego's route and engage come from `acb_pilot` in the ego's domain, exactly as 010
      plans for background AVs — one mechanism, N consumers
- [ ] The ego's goal pose comes from bridge config. This is a real authoring cost: a
      scenario's `AcquirePositionAction` on the ego no longer reaches Autoware, so the goal
      is stated twice (once in the `.xosc` for SSv2's conditions, once in config for the
      pilot). Document it loudly rather than hiding it
- [ ] A pilot that fails to engage is reported by the bridge — visibly, not as a scenario
      that mysteriously times out

### Launch

- [ ] `ego_av.launch.xml` (or a generalisation of `background_av.launch.xml`) brings up
      Autoware + `acb_bridge` + pilot for the ego in its own domain, `publish_clock:=true`
- [ ] The scenario launch sets `launch_autoware:=false` and drops
      `autoware_launch_package` / `autoware_launch_file` / `sensor_model` / `vehicle_model`
      — SSv2 no longer needs to know how Autoware is launched
- [ ] Delete the concealer `launch.hpp` patch and the `PLAY_LAUNCH_WEB_ADDR` plumbing
      (gap 10 becomes moot)
- [ ] Startup order documented: the ego stack comes up like any background stack, before
      SSv2, waiting on its `role_name`

### Clock symmetry

- [ ] With the ego's Autoware out of SSv2's domain, `acb_bridge` publishes `/clock` in the
      ego's domain like every other domain — the `publish_clock:=false` special case is
      deleted, not defaulted
- [ ] Verify nothing in SSv2's own domain needs a `/clock` beyond what SSv2 itself publishes

### Documentation

- [ ] Rewrite Design Principle 5 in [architecture.md](../design/architecture.md): SSv2
      drives the *scenario*, not Autoware
- [ ] Update the roles table and startup sequence in
      [multi-instance-architecture.md](../design/multi-instance-architecture.md); delete the
      "Known conflict: AutowareUniverse vehicle status" section once it is resolved by
      construction
- [ ] Rewrite [ssv2-launch-configuration.md](../design/ssv2-launch-configuration.md) around
      the unmanaged launch; the AutowareUniverse mitigation section goes away
- [ ] Record the scenario-authoring constraint: ego autonomy actions in `.xosc` are limited
      to what the pilot implements

### Tests

- [ ] Unit: ego config carries a goal pose; missing goal with an ego present fails at
      startup, not at engage time
- [ ] Integration: a run with `launch_autoware:=false` spawns the ego, ticks, and evaluates
      a pose-based condition
- [ ] End-to-end: single-ego scenario reaches `exitSuccess` with no concealer process alive
      anywhere on the host
- [ ] End-to-end: exactly one `/clock` publisher in every domain, none silenced by
      configuration

## Acceptance Criteria

- [ ] No process forked by SSv2 exists during a scenario run — `ps` shows Autoware only
      under our launch files
- [ ] `/vehicle/status/*` has exactly one publisher in the ego's domain (`acb_bridge`)
- [ ] The SSv2 checkout carries zero local patches
- [ ] Ego and background AV stacks use the same launch file, pilot, and clock policy,
      differing only in config
- [ ] A scenario in which Autoware fails to engage produces a bridge-side error naming the
      pilot, not a bare SSv2 timeout
- [ ] `just test` passes

## Risks

- **SSv2 may not tolerate an unmanaged ego at all.** That is what the spike is for; the
  answer bounds the phase before any launch surgery starts.
- **Scenario expressiveness shrinks.** Conditions over Autoware internals (engage state,
  MRM state) stop working. Scenarios that only condition on poses, distances, collisions
  and traffic lights — everything this bridge feeds back — are unaffected.
- **The pilot becomes load-bearing for every run**, and 010 records that it has never been
  wired up. Its integration must land (010's open pilot item) before this phase can be
  verified end-to-end.
