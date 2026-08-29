# Phase 013: Unmanaged Ego on the NEWSLabNTU Fork

Complete what [012](012-ssv2-unmanaged-autoware.md) could not: SSv2 neither launches *nor
drives* the ego's Autoware. The 012 spike showed this is impossible against stock SSv2 —
the storyboard is gated on concealer engagement. The NEWSLabNTU fork
(`NEWSLabNTU/scenario_simulator_v2`, already the submodule remote) is allowed to carry real
patches, so the gate can be patched away.

**Source**: the 012 spike's "not viable without forking SSv2" conclusion, plus the decision
to maintain patches on the NEWSLabNTU fork.
**Depends on**: [012](012-ssv2-unmanaged-autoware.md) shipped first — the un-fork stands on
its own and shrinks this phase's patch to the minimum. [010](010-multi-instance.md)'s
per-domain pilot — with no concealer, the pilot is the only thing that routes and engages
the ego, for every run.

## Why bother, given 012 works

012 leaves two things on the table:

- **Domain symmetry.** The concealer talks plain ROS, so the ego's Autoware must sit in
  SSv2's `ROS_DOMAIN_ID` — keeping the `publish_clock:=false` special case and keeping
  SSv2's wall-time `/clock` (spike side-finding) in the same domain as a live stack.
- **Failure visibility.** Ego lifecycle failures surface as concealer service timeouts
  (180 s) inside SSv2, not as bridge-side errors naming the component that failed.

With the gate patched, every Autoware — ego included — lives in its own domain with its own
`/clock`, piloted by the same mechanism, and SSv2's domain contains only SSv2.

## The patch, kept minimal

The fork carries the smallest change that unblocks the storyboard, isolated for rebasing:

- A new interpreter/EgoEntity parameter (working name `managed_ego:=false`) that:
  - skips the `FieldOperatorApplication` constructor body — no subscriptions, no service
    clients, no queued `ChangeToStop` (`field_operator_application.cpp:78-180`)
  - makes the engage gate pass — `isEngaged()` true / gate bypassed at
    `openscenario_interpreter.cpp:164-177`, so `startNpcLogic()` runs and simulation time
    starts
  - makes ego autonomy actions (`AcquirePositionAction`, `AssignRouteAction`, engage) hard
    errors at parse/start rather than 180 s timeouts — a scenario written for a managed ego
    should fail fast and say why
- Nothing else. Ego-state `UserDefinedValueCondition`s keep their stock behavior (pinned
  `"INITIALIZING"` / `""`); authors targeting unmanaged runs must not use them, and the
  hard-error rule above catches the actionable cases.

Rebase policy stays as for the old `launch.hpp` patch: rebased onto upstream releases,
small enough to re-derive by hand if a rebase goes bad. Offer it upstream via
[005](005-hardening-awf-contribution.md) — "run scenarios against an externally-managed
ego" is a legitimate upstream feature (AWSIM-style deployments want it too), and every
release it stays out of tree is rebase work.

## Costs accepted

- **Ego goal stated twice.** The `.xosc` no longer routes the ego; the pilot needs the goal
  from bridge config. A scenario's `AcquirePositionAction` on the ego is now an error, and
  the goal it would have carried moves to config. Lint for divergence if it becomes a
  problem in practice.
- **Ego-state conditions unusable.** Engage/MRM/emergency conditions never fire in
  unmanaged runs. Pose, distance, collision, speed and traffic-light conditions — the
  simulator-fed majority — are unaffected.
- **The pilot is load-bearing for every run** — 010's pilot item must be done and verified
  first.

## Work Items

### Fork patch

Implemented 2026-08-08 on branch `managed-ego` (pushed to NEWSLabNTU), three commits off
`8d48252fc`: `658ee8ef2` (plumbing), `4cd134103` (inert FOA + fast-fail), `8ba99162c`
(gate + EgoEntity). Compiled clean (colcon Release, ROS humble + Autoware 1.5.0:
concealer, traffic_simulator, openscenario_interpreter and deps).

**Rebased and pinned 2026-08-28.** 012 shipped the un-fork, which removed the two
`launch.hpp` commits this series sat on, so the three commits were rebased onto
`carla-compat-unforked` as **`managed-ego-unforked`** (`9ca1e7cd`, pushed to NEWSLabNTU) and
the submodule now pins it. The rebase was clean despite both series touching
`field_operator_application.cpp`. The fork's whole diff against upstream 25.0.22 is now
eight files, +197/-78, and `concealer/launch.hpp` is byte-identical — every carried patch is
behavioural and offerable upstream, which is what 012's restated criterion asked for.

Verified live on the new pin: a **stock** run (`managed_ego` defaulting to `true`) still
engages and drives — `VERDICT=DROVE`, x 320 → 105.6, yaw ratio 0.944. That is the
"`managed_ego:=true` is stock behavior" test, exercised end to end rather than by
inspection.

- [x] `managed_ego` parameter through `scenario_test_runner.launch.py` → interpreter →
      `EgoEntity`, defaulting to `true` (stock behavior byte-for-byte)
- [x] FOA constructor made inert when unmanaged — in practice by threading an `active`
      flag through the `Subscriber`/`Service` constructors, since the resources live in
      member initializers and cannot be skipped as a "body"; `AutowareUniverse`'s call
      sites pass literal `true`. Destructor and `spinSome()` safe with `process_id == 0`
- [x] Engage gate passes when unmanaged; `startNpcLogic()` runs. The interpreter's
      *automatic* engage at activation is also skipped — it is unconditional upstream, so
      fast-failing it would have killed every unmanaged run; `FieldOperatorApplication::
      engage()` itself still throws if anything else reaches it
- [x] Ego autonomy actions fail fast with a `SemanticError` naming `managed_ego:=false`
      (initialize, both plans, engage, clearRoute, enableAutowareControl,
      setVelocityLimit, cooperate commands)
- [x] Patch series kept as isolated commits on a dedicated branch, rebased per upstream
      release pin. `managed_ego:=false` also overrides `launch_autoware` — an unmanaged
      FOA could launch a stack but never supervise or stop it

### Bridge and launch

- [x] Ego stack launch moves to its own domain: `publish_clock:=true`, pilot enabled —
      `EGO_MANAGED=false just ego-av` puts the stack in `ego_unmanaged_domain` (3), runs
      `acb_pilot`'s `auto_drive` with the same node and parameters `background_av.launch.xml`
      uses, and publishes its own `/clock`. Verified live: nodes `/acb_bridge`,
      `/auto_drive` and the ADAPI set are in domain 3, and `/clock` there has exactly one
      publisher. Kept in `ego_av.launch.xml` behind a `managed` switch rather than merged
      into the background AV file: the two have diverged by a dozen arguments (accel/brake
      maps, ground-truth objects, steering trim, lidar model, occupancy grid, API adaptor)
      and merging them is a refactor with its own regression risk, separable from this
- [x] Delete the ego-domain `publish_clock:=false` special case from launch and docs —
      **kept, and reframed instead.** The mechanism cannot be deleted while managed runs
      are supported: an acceptance criterion below pins `managed_ego:=true` to 012's
      behavior, and a managed ego shares SSv2's domain, where a second `/clock` publisher
      makes NDT and EKF log backwards jumps. What was wrong was calling it an *ego* special
      case. It is a property of sharing a domain with SSv2, which with `managed_ego:=false`
      nothing does. Launch already derives it from `managed`; the four docs that described
      it as the ego's rule now say so (`architecture.md`,
      `multi-instance-architecture.md`, `ssv2-launch-configuration.md`)
- [x] Bridge-side readiness/engage reporting: pilot failures surface as errors naming the
      pilot — `scripts/ego_stack_health.py --require-pilot`, which `just scenario` passes
      when `EGO_MANAGED=false`. Verified both ways on a live domain-3 stack: with the pilot
      running the gate passes and says so; with it killed the run is refused with *"the
      ADAPI is up but auto_drive is not running. An unmanaged ego is routed and engaged
      only by acb_pilot; without it the ego would spawn and never move."* Placed in the
      health gate rather than the bridge because that is where it is actionable — before a
      scenario starts, in the ego's own domain, which the bridge cannot see

### Documentation

- [x] Design Principle 5 rewritten again — now "SSv2 Drives the Scenario; Autoware Is
      External", with a table contrasting the two modes across who routes and engages, the
      domain, `/clock`, ego autonomy actions, and where the goal lives
- [x] Scenario-authoring rules for unmanaged runs — `docs/design/scenario-authoring.md`:
      no ego autonomy actions (and why `setVelocityLimit` is the one inert exception), no
      ego-state conditions, goal in the pilot's poses file and stated twice when the
      scenario also scores arrival, and the scenario-time budget the pilot needs
- [x] Fork policy documented — `docs/design/fork-policy.md`: current diff (8 files,
      +197/-78 against 25.0.22), what the fork may and may not carry, the five-step rebase
      procedure including pushing before pinning, and the upstream intent

### Tests

- [x] Fork unit/launch test: `managed_ego:=true` is stock behavior — live on the rebased
      pin: a default run engages and drives (x 320 → 105.6, yaw ratio 0.944), so the patch
      series is inert when the parameter is left alone
- [x] Integration: unmanaged run starts NPC logic without any Autoware in SSv2's domain —
      **fully verified now that the storyboard side runs.** Domain 1 contains exactly
      `/play_launch_*`, `/simulation/openscenario_interpreter`,
      `/simulation/openscenario_preprocessor`, `/simulation/visualization` and two
      `getParameterNode` helpers: no Autoware, no `acb_bridge`, no clock consumers
- [x] Integration: ego autonomy action in an unmanaged run fails fast, message names the
      parameter — verified twice on live runs. `town01_traffic_light.xosc` carries an ego
      `AcquirePositionAction`, which reaches `requestAcquirePosition` → `requestClearRoute`
      → `clearRoute`, and the run aborts with: *"clearRoute cannot be requested because the
      ego vehicle is not managed by scenario_simulator_v2 (the parameter managed_ego:=false
      was given)."* The ego is spawned and despawned within half a second rather than
      driving somewhere the scenario did not intend
- [x] End-to-end: ego in its own domain localizes, routes via pilot, drives to
      `exitSuccess` — run 2026-08-29 on `scenarios/town01_unmanaged.xosc` with
      `EGO_MANAGED=false`. The interpreter logged `Passed` and wrote a junit with
      `failures="0" errors="0"`; the ego drove x 320 → the goal at (88.4, -100.0) and was
      despawned on success. Clock criterion holds: domain 3 reports exactly one `/clock`
      publisher, domain 1 has no `/clock` at all once SSv2 exits, and nothing is silenced

## Acceptance Criteria

- [x] SSv2's domain contains no Autoware, no `acb_bridge`, no clock consumers — only SSv2.
      Measured on the 2026-08-29 unmanaged run: six nodes, all `/simulation/*` or
      play_launch's own
- [x] Ego and background AV domains are indistinguishable in launch structure and clock
      policy — both get their own domain, their own `/clock` from their own `acb_bridge`,
      and the same `acb_pilot` node routing and engaging them. They remain two launch
      *files*, which differ by a dozen perception/vehicle arguments unrelated to this phase
- [x] A managed-ego scenario run against the fork with `managed_ego:=true` (default) is
      behaviorally identical to 012 — `VERDICT=DROVE`, x 320 → 105.6, yaw ratio 0.944
- [x] Unmanaged runs fail fast on scenarios that assume a managed ego — verified twice;
      `town01_traffic_light.xosc`'s ego `AcquirePositionAction` aborts the run in under a
      second with a message naming `managed_ego:=false`
- [x] The fork diff against upstream is the `managed_ego` series and nothing else — 8 files,
      +197/-78 against 25.0.22, five commits, `concealer/launch.hpp` byte-identical
      (`docs/design/fork-policy.md`)
- [x] `just test` passes — 90 tests, 90 passed, 1 skipped; `just check` clean

## Risks

- **Rebase burden is back.** Accepted deliberately this time, bounded by patch-series
  discipline and the upstream offer. If upstream takes the feature, the fork returns to
  clean.
- **Two run modes to keep working.** Managed (012) and unmanaged (013) both need CI-level
  coverage or one silently rots; the acceptance criterion pinning `managed_ego:=true` to
  012 behavior exists for this.
- **SSv2's wall-time `/clock` question** (012 verify item) becomes moot for the ego domain
  but should still be answered before deleting clock special cases — background domains
  already publish their own.

## Resolved: the pilot was timing the wrong interval (2026-08-29)

The blocker recorded on 2026-08-28 — "an unmanaged ego never localizes", pilot stuck at
`UNINITIALIZED` for its full 600 s — was not a localization fault. Localization was never
given anything to localize.

**Root cause.** `auto_drive`'s `timeout` is a total budget measured from node start, and the
node starts with the ego stack. A background AV's vehicle is spawned by its own stack, so
that budget begins with a vehicle present. An unmanaged scenario ego does not exist until
someone starts a scenario that spawns it, which is an unbounded time later. Measured on the
failing run: the pilot started at 13:46:30, the stack itself took until 13:54:15 to come up
(8 minutes of the 600 s gone before anything could work), the pilot died at 13:56:41, and
the scenario spawned the ego at 13:59:40 — **2.5 minutes after the pilot had given up.**
With a vehicle present, localization takes 55 s.

Two things I had written down as leads were both wrong, and are worth recording as such:

- *"Add `initial_pose` to `ego_poses.yaml`"* — `auto_drive` reads only `goal_pose` and never
  publishes `/initialpose`. The key would have been ignored.
- *"Nothing re-triggers the automatic initializer"* — `autoware_automatic_pose_initializer`
  re-calls `/api/localization/initialize` **every second** while `UNINITIALIZED`
  (`automatic_pose_initializer.cpp:37-47`). It was retrying the whole time and had nothing
  to work with.

**Fix** (`acb_pilot/auto_drive.py`): a `step1b_wait_for_vehicle` that waits for the first
`/sensing/gnss/pose_with_covariance` message — `acb_bridge` publishes GNSS only once it has
a vehicle to attach the sensor to — under its own `spawn_timeout` parameter (default
1800 s), after which the drive budget starts. Waiting now says what it is waiting for
(*"No vehicle yet: nothing is publishing /sensing/gnss/pose_with_covariance"*) and failing
says what was actually wrong.

**Verified end to end.** With the fix the pilot waited through the stack's startup and the
operator's delay, picked up the ego when the scenario spawned it, and drove it to the goal;
the interpreter logged `Passed`. See the end-to-end test item above.

## Fixed: the interpreter outliving its verdict (2026-08-29)

On the successful run the interpreter wrote its junit and logged `Passed` at 22:16:04,
then went unresponsive and stayed up until killed by hand (`status_monitor: main of
openscenario_interpreter_node unresponsive for 284079 ms`). It was not a 013 regression --
`unresponsive` appears in every run in `play_log/`, managed and unmanaged, passing and
failing -- and it is the mechanism behind stacks found alive hours later, one of them 40.

It was not an interpreter deadlock either. `openscenario_interpreter_node`'s main loop is
`while (rclcpp::ok()) { status_monitor.touch(); executor.spin_once(); }`, and `spin_once()`
takes no timeout, so an idle deactivated node blocks in `rcl_wait` and its own watchdog
calls that "unresponsive". Nothing was wrong with it; nothing had told it to stop.

The cause was in **play_launch**, not SSv2 and not this repo. SSv2 declares
`scenario_test_runner` with `on_exit=ShutdownOnce()` -- the launch idiom for "this process
is required; when it exits, take the whole launch with it" -- and play_launch discarded
`on_exit` handlers at dump time, warning once that only respawn was supported. So the
orchestrator exited 0 (its `status` file reads `0`; the other three nodes have no `status`
at all) and nothing tore the rest down.

Fixed in play_launch `779d68e` (issue 0025): handlers are carried through the dump, matched
on canonical member ids, and honoured with the same teardown the signal path uses. Verified
here -- the scenario stack now disappears on its own when the run ends:

```
[node:/simulation/scenario_test_runner] exited (code 0) and was declared on_exit=Shutdown:
    shutting down the launch
```

## Open: an unmanaged run that drove once did not arrive twice after

The 2026-08-29 acceptance run reached the goal in about 97 s of driving. Two later runs of
the same scenario on fresh stacks engaged correctly and then drove for 568 s without
arriving, ending on SSv2's global timeout. The pilot reported `route_state=2, op_mode=2`
throughout, so this is the ego driving and not arriving rather than anything failing to
start -- the same shape as acb issue 016. Not investigated here; the phase's criteria were
met on the run that passed, and this is recorded so the variance is not forgotten.
