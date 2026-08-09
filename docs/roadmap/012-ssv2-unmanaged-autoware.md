# Phase 012: SSv2-Unmanaged Autoware

SSv2 stops *launching* the ego's Autoware. Every Autoware stack — ego and background alike —
is brought up by our launch files, and the SSv2 checkout carries zero local patches. SSv2
keeps driving the ego's autonomy (initialize, route, engage) through the concealer, which
connects to the externally-launched stack instead of forking one.

**Source**: narrows Design Principle 5 of [architecture.md](../design/architecture.md);
removes gap 10 (the concealer `launch.hpp` patch) entirely.
**Depends on**: nothing outside this phase. The spike removed the expected dependency on
010's pilot — see below.

## Spike results (2026-08-08)

Verified against the `src/scenario_simulator_v2` submodule: upstream 25.0.22 plus two local
commits touching only `external/concealer/include/concealer/launch.hpp`.

### `launch_autoware:=false` means "don't fork", not "run without Autoware"

`EgoEntity` inherits `concealer::FieldOperatorApplication` as a base class
(`ego_entity.hpp:34`) — the concealer is always constructed. The parameter decides exactly
one thing (`ego_entity.cpp:72-77`): whether the constructor gets a real child pid from
`concealer::ros2_launch(...)` or `0`. A zero pid skips the fork, the `waitpid` liveness
check, and the shutdown signalling. Every subscription, service client, and state machine
stays fully wired. The concealer's own source states the intended use: *"In case of reusing
the same Autoware instance for multiple scenarios (launch_autoware:=False), we need to
ensure that Autoware is in a safe STOP state before starting the next scenario"*
(`field_operator_application.cpp:171-180`).

So the supported configuration is: **an Autoware already running in SSv2's ROS domain,
launched by someone else**. The concealer finds it, initializes localization, sets the route
from the scenario, and engages — exactly as today, minus the fork.

### Running with *no* Autoware at all is not viable without forking SSv2

With `launch_autoware:=false` and no Autoware present, a scenario does spawn the ego over
ZMQ (`is_ego=true`), does tick `UpdateFrame` — and never evaluates a single condition. The
storyboard is gated on ego engagement (`openscenario_interpreter.cpp:164-177`):
`startNpcLogic()` waits for `isEngaged()`, which requires the concealer's state machine to
reach `driving`. With no Autoware, every concealer subscriber returns a default-constructed
message, the legacy state pins at `initializing`, simulation time stays NaN, and the
storyboard is never reached. The run then dies cleanly ~180 s in, when the
`requestChangeToStop` queued at construction exhausts the service-availability timeout
(`service.hpp:53-63`) and surfaces as `AutowareError` at the next `spinSome()`.

Constructs that would break even if that gate were forked away: ego
`AcquirePositionAction`/`AssignRouteAction` (Autoware ADAPI services, 180 s timeout →
throw), `UserDefinedValueCondition` on ego state (`currentState` pinned `"INITIALIZING"`,
MRM/emergency states silently `""` forever), RTC cooperate commands (throw synchronously).
Pose/distance/collision/traffic-light conditions are all simulator-fed and safe.

Conclusion: full unmanagement is a patch-sized change to SSv2 (make the engage gate and the
concealer optional per ego), not a launch-configuration change. This phase therefore
delivers the **un-fork**; full unmanagement is [013](013-forked-unmanaged-ego.md), carried
on the NEWSLabNTU fork and offered upstream via [005](005-hardening-awf-contribution.md).

### What the un-fork wins

- `concealer::ros2_launch()` is never called, so our `launch.hpp` patch — the only local
  SSv2 modification — becomes dead code and is deleted. Gap 10 closes; the submodule pins
  clean upstream.
- The ego's Autoware comes up from our launch files with the same lifecycle, logging, and
  restartability as every background AV. SSv2 no longer needs
  `autoware_launch_package`/`autoware_launch_file`; the concealer never spawns a process it
  can kill on scenario end.
- No scenario-expressiveness loss and no goal-pose duplication: the concealer still routes
  from the `.xosc` and still engages.

### What the un-fork does not win

- **No domain symmetry.** The concealer reaches Autoware over plain ROS topics/services, so
  the ego's Autoware must live in SSv2's `ROS_DOMAIN_ID`. The `publish_clock:=false`
  special case for the ego's `acb_bridge` stays.
- **SSv2 still owns the ego's autonomy lifecycle.** Initialize/route/engage remain concealer
  calls. This phase renames Principle 5's mechanism, not its authority.

### First live run (2026-08-08, partial — shared GPU kept killing CARLA)

The full stack ran on this path for the first time: CARLA (offscreen, VRAM-tuned) +
`csb_bridge` + `ego_av.launch.xml` + `just scenario` with `launch_autoware:=false`.
Verified live before the shared GPU's training job OOM-killed CARLA (four times):

- SSv2 → ZMQ `Initialize` → Town01 load → **36/36 traffic lights matched** on the
  NEWSLab map pack → freeze → ego spawn (`role_name=hero`) → sync mode → `acb_bridge`
  attached all four sensors.
- **The concealer drove the externally-launched Autoware**: it connected, called
  `/api/autoware/set/velocity_limit` and `/api/operation_mode/change_to_stop` — the
  un-fork works end to end up to engagement. Engage/drive/`exitSuccess` remain
  unverified; CARLA died under the stack first.
- Startup order works as documented: ego stack first, SSv2 second, concealer finds it.
- `use_sim_time` on the SSv2 launch **must stay false** (the default): with true, the
  interpreter's ROS clock waits for a `/clock` that only SSv2 itself would publish
  after activation, and `main()` blocks before the scenario starts. So SSv2's `/clock`
  carries wall time by construction; the ego domain's Autoware (`use_sim_time=true`)
  consumes it together with sensor stamps from the same clock, consistently.

Fixes that landed on the way: SSv2 rejects the map pack's `LocalCartesianUTM`
projector (`download_maps.sh` now rewrites it to TransverseMercator);
`ego_av.launch.xml` launches `autoware_iv_external_api_adaptor` (nothing in the acb
profile provided `/api/autoware/set/velocity_limit` — a silent 180 s timeout
otherwise); `csb_bridge` probes and rebuilds a dead CARLA connection at `Initialize`
(tick-driven reconnect never fires between scenarios); play_launch needs
`--parser python` for both launches and chokes on empty-string parameters
(`record_storage_id`).

### Second live session (2026-08-08, evening): chain verified through routing

Thirty-six scenario attempts, each failure fixed in the responsible component. The
concealer now drives the externally-launched Autoware through: connect →
`ChangeToStop` → `setVelocityLimit` → **localization initialize (accepted)** →
**`set_route_points` (route SET)** → PLANNING. Engagement remains blocked — see below.

What it took, by component:

- **play_launch** (branch `fix/compound-parameter-serialization` + installed 0.8.2):
  empty-string params crashed rcl (fixed upstream between 0.5.1 and HEAD); dict
  params as Python reprs crashed `pose_initializer` fatally; array params were
  retyped to strings and rejected by statically typed declarations. Both fixed.
- **DDS on this host**: `lo` multicast is off and NIC-multicast discovery loses a
  random service almost every run at ~170 participants. `config/
  cyclonedds-localhost.xml` pins loopback unicast with explicit peers,
  `MaxAutoParticipantIndex` 300 (every composable takes an index) and
  `SPDPInterval` 5 s (default resend backoff exceeds the concealer's 180 s service
  timeout for late joiners). Deterministic since. All recipes export it; zombie
  `play_launch`/`visualization_node` processes from killed runs must be purged or
  they exhaust participant indices and ghost-list dead services.
- **Autoware profile**: `/api/autoware/set/velocity_limit` is served by the
  *internal* API adaptor (`autoware_iv_internal_api_adaptor`), which nothing
  launched — the concealer's call sat in a 180 s wait, and after that in
  30 rejected retries against the external adaptor's lookalike. Both adaptors now
  launch with `just ego-av`. `component_state_monitor` rates/timeouts relaxed for
  a sub-realtime sim (acb `6fefaa6`).
- **Scenario**: the original goal is unreachable in the NEWSLab Town01 routing
  graph ("The planned route is empty"); re-aimed along the spawn street.
  `global_frame_rate` 30 → 10 (the host cannot hold 30), `initialize_duration`
  120 → 480 (the concealer's deadline is absolute from construction and shared by
  every wait through engage).
- **acb_bridge**: does not notice when SSv2 despawns the ego, so its sensors die
  with the actor and the *next* scenario starves — restart it (or the whole ego
  stack) between runs until vehicle-respawn redetection lands (roadmap 011).
- **CARLA**: five crashes traced to the 0.9.16 sensor-stream teardown race
  (`Invalid session: no stream available`), not only VRAM; both bridges survive it
  (carla-rust `2718365`) and `systemd-run -p Restart=on-failure` (unit
  `carla-e2e`) makes it self-healing.

**Remaining blocker — engagement**: `is_autonomous_mode_available` never turns
true; the ego stalls in PLANNING with route SET. Mid-stall inspection shows the
diagnostics gate red on localization continuity (`pose_twist_fusion_filter/pose`
ERROR) while clustering perception produces objects.

The initially suspected cause — pcd map fidelity — is **ruled out** (2026-08-09):
the NEWSLab Town01 pcd is 16.7M points with extent x[-51.6, 446.2],
y[-379.3, 40.2], matching the acb-verified TUM map's documented extent exactly
(same lineage), it is dense at both spawn and goal, and NDT *initial* alignment
succeeded live against it. The prime suspect is now a **clock discontinuity**:
NDT logs "Detected jump back in time. Clearing TF buffer", consistent with
SSv2's wall-time `/clock` stopping and restarting across scenario runs while
the ego stack persists — single-shot alignment survives that, history-dependent
EKF fusion does not. Next step: one scenario against a completely fresh stack
(no prior run, hence no clock transition) while watching
`/localization/kinematic_state` and NDT health topics mid-run; the outcome
decides between clock-transition handling and NDT convergence profiling.

### Verify live (side findings, not blockers)

- SSv2 publishes `/clock` unconditionally at frame rate (`api.hpp:64-66`,
  `api.cpp:146-147`), but with the launch default `use_sim_time:=false` it carries **wall
  time** (`simulation_clock.cpp:40-47`). Confirm what our launch actually passes and what
  the ego domain's Autoware consumes — a wall-time `/clock` next to sim-time-stamped sensor
  data would be a real inconsistency, and today's docs assume SSv2's `/clock` is
  authoritative sim time.
- The `AutowareUniverse` vehicle-status dual-publisher conflict
  ([ssv2-launch-configuration.md](../design/ssv2-launch-configuration.md#autowareuniverse-topic-conflict))
  may be moot already: that node is constructed inside the `simple_sensor_simulator`
  process (`simple_sensor_simulator.cpp:201-215`), which we launch with
  `launch_simple_sensor_simulator:=false`. The `FieldOperatorApplication` node in the
  interpreter process only subscribes. Check a live run for a second
  `/vehicle/status/*` publisher; if absent, delete the conflict sections from both design
  docs.
- `sensor_model` / `vehicle_model` parameters have no defaults and are read before the
  `launch_autoware` branch (`ego_entity.cpp:64-65`) — they must stay in the SSv2 launch
  arguments even though SSv2 no longer launches Autoware.

## What does not change

- The ZMQ protocol and all 14 handlers. SSv2 still spawns the ego via
  `SpawnVehicleEntity(is_ego=true)` and reads its pose back each frame.
- Pose authority: ego stays CARLA PhysX; NPCs stay teleported.
- Scenario semantics: routing, engagement, and every condition work exactly as today.
- Sensors, NPC puppeteering, traffic lights.

## Work Items

### Investigation first (spike)

- [x] What does `launch_autoware:=false` do? → "Don't fork; use the Autoware already in my
      domain." Spawn/tick/conditions all work when such an Autoware exists; see spike
      results above.
- [x] Which scenario constructs die without a concealer-connected Autoware? → All ego
      autonomy actions and ego-state conditions; catalogued above. Moot for this phase since
      the concealer stays connected.
- [x] Does SSv2 still publish `/clock` with no Autoware forked? → Yes, unconditionally;
      wall-time by default. Special case stays; see verify-live list.

### Launch

- [x] `ego_av.launch.xml` brings up Autoware + `acb_bridge` for the ego **in SSv2's
      domain**, `publish_clock:=false`, reusing the structure of
      `background_av.launch.xml` — one launch mechanism, domain and clock policy as
      config. `just ego-av` runs it; `demo.launch.xml` includes it in place of its bare
      `acb_bridge` include.
- [x] The scenario launch sets `launch_autoware:=false` and keeps `sensor_model` /
      `vehicle_model` (still read by `EgoEntity` before the branch); drop
      `autoware_launch_package` / `autoware_launch_file`
- [ ] Startup order documented and enforced where possible: the ego stack must be up and
      Autoware's ADAPI services available before SSv2 spawns the ego — the concealer's
      constructor queues a `ChangeToStop` with a 180 s service timeout, so a late stack
      turns into a slow, confusing failure. *Documented (ssv2-launch-configuration.md,
      launch-file comments); no mechanical enforcement yet — needs a live run to pick a
      readiness check that actually gates.*
- [ ] Scenario-end semantics checked: with no child process, SSv2 cannot kill Autoware on
      exit; the ego stack persists across scenario runs. Verify the concealer's
      stop-state reset actually returns the reused Autoware to a re-engageable state for
      the next scenario (this is the upstream-intended flow, but it has never run here)

### Delete the fork machinery

> **Superseded in part**: the SSv2 fork now carries phase 013's `managed_ego` series, so
> "pin clean upstream" no longer applies. The `launch.hpp` fork-machinery commits still
> get dropped, but via 013's branch (rebased without them), not a clean-upstream pin.

- [ ] Drop the two `launch.hpp` fork-machinery commits from the SSv2 submodule — via
      013's `managed_ego` branch, which replaces the old "pin clean upstream 25.0.22"
- [ ] Remove `PLAY_LAUNCH_WEB_ADDR` plumbing from our launch files and docs
- [ ] Close gap 10 in [multi-instance-architecture.md](../design/multi-instance-architecture.md)

### Documentation

- [ ] Rewrite Design Principle 5 in [architecture.md](../design/architecture.md): SSv2
      drives the ego's *autonomy* (initialize, route, engage); it launches nothing
- [ ] Update [ssv2-launch-configuration.md](../design/ssv2-launch-configuration.md):
      `launch_autoware:=false`, the ego stack's launch file, startup order; resolve or
      delete the AutowareUniverse conflict section per the live check. *Done except the
      conflict section, which waits on the live check.*
- [ ] Update the roles table and startup sequence in
      [multi-instance-architecture.md](../design/multi-instance-architecture.md)
- [ ] Record the reuse semantics: one long-lived ego stack across scenario runs, reset to
      stop state between them. *Recorded in ssv2-launch-configuration.md; the reset flow
      itself is still unverified (see Launch item above).*

### Tests

- [ ] Integration: scenario run with `launch_autoware:=false` against a pre-launched stack
      reaches the storyboard (conditions evaluate; sim time is not NaN)
- [ ] Integration: two consecutive scenario runs against the same ego stack both engage
- [ ] End-to-end: single-ego scenario reaches `exitSuccess` with `ps` showing Autoware only
      under our launch files, none under SSv2
- [ ] Regression guard: SSv2 submodule pin is reachable on upstream (no local commits)

## Acceptance Criteria

- [ ] No process forked by SSv2 exists during a scenario run
- [ ] The SSv2 checkout carries zero local patches
- [ ] Ego and background AV stacks share launch structure, differing only in domain and
      clock config
- [ ] A scenario retains full expressiveness: ego routing from `.xosc`, engage-state
      conditions work
- [ ] Consecutive scenario runs reuse the ego stack without a restart
- [ ] `just test` passes

## Risks

- **Reuse-reset is unproven here.** The concealer's between-scenarios stop-state flow is
  upstream-intended but has never run in this project; if Autoware does not return to a
  re-engageable state, scenario N+1 fails in ways scenario N never showed. The
  two-consecutive-runs test exists to catch exactly this.
- **Startup ordering becomes operator-visible.** Today SSv2 launches Autoware at the right
  moment by construction; now a scenario started before the ego stack is ready fails 180 s
  later with a service timeout. Mitigate with a readiness check in the scenario launch
  wrapper.
- **The wall-time `/clock` question.** If verification shows SSv2's `/clock` has been wall
  time all along, the ego domain's clock story needs rework independent of this phase —
  surface it, don't bury it.
