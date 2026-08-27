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
eight files, +182/-78, and `concealer/launch.hpp` is byte-identical — every carried patch is
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

- [ ] Ego stack launch moves to its own domain: `publish_clock:=true`, pilot enabled, same
      file as background AVs with different config
- [ ] Delete the ego-domain `publish_clock:=false` special case from launch and docs
- [ ] Bridge-side readiness/engage reporting: pilot failures surface as bridge errors
      naming the pilot (carried over from the original 012 draft)

### Documentation

- [ ] Design Principle 5 rewritten again: SSv2 drives the *scenario*; Autoware lifecycle
      and autonomy are external, uniformly
- [ ] Scenario-authoring rules for unmanaged runs: no ego autonomy actions, no ego-state
      conditions; goal lives in bridge config
- [ ] Fork policy documented: what the fork may carry, rebase cadence, upstream intent

### Tests

- [x] Fork unit/launch test: `managed_ego:=true` is stock behavior — live on the rebased
      pin: a default run engages and drives (x 320 → 105.6, yaw ratio 0.944), so the patch
      series is inert when the parameter is left alone
- [ ] Integration: unmanaged run starts NPC logic without any Autoware in SSv2's domain
- [ ] Integration: ego autonomy action in an unmanaged run fails fast, message names the
      parameter
- [ ] End-to-end: ego in its own domain localizes, routes via pilot, drives to
      `exitSuccess`; every domain has exactly one `/clock` publisher, none silenced

## Acceptance Criteria

- [ ] SSv2's domain contains no Autoware, no `acb_bridge`, no clock consumers — only SSv2
- [ ] Ego and background AV domains are indistinguishable in launch structure and clock
      policy
- [ ] A managed-ego scenario run against the fork with `managed_ego:=true` (default) is
      behaviorally identical to 012
- [ ] Unmanaged runs fail fast on scenarios that assume a managed ego
- [ ] The fork diff against upstream is the `managed_ego` series and nothing else
- [ ] `just test` passes

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
