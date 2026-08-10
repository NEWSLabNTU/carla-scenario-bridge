# Session checkpoint — 2026-08-10

State of the SSv2+CARLA integration after the exitSuccess campaign. For the full
per-phase anatomy see `docs/roadmap/012-ssv2-unmanaged-autoware.md`; this file is
the cross-machine handoff summary.

## Update — 2026-08-10, second machine (newslab-server243, RTX 5090)

The stack was brought up from a clean clone on a second host and the two-domain run
now goes further than it ever has:

- **Ego E2E green here too**: `town01_ego_drive.xosc` passed five consecutive times
  (SSv2 JUnit clean), spawn-to-goal ~35-50 s.
- **010's last open leg is observed**: the background AV's pilot **set its route and
  engaged** inside the scenario window — `Route set successfully`, then
  `Driving... route_state=2, op_mode=2` (AUTONOMOUS) 46 s after the AV spawned, while
  the ego was still driving. It did not *arrive*: the world only ticks while SSv2 runs
  a scenario, the ego's scenario is ~50 s long, and the pilot needs ~45 s of that just
  to localize (GNSS+NDT) and settle. The bg AV therefore gets a few seconds of
  autonomous driving per run. Making it arrive needs either a longer ego scenario or a
  shorter pilot cold start, not a fix to the mechanism.
- **Teardown is clean**: after each run CARLA holds no vehicles or sensors, background
  AV included (010's "no background AV left behind").

### What it took to get here (all of it new, all of it committed)

1. **play_launch must carry the compound-parameter fix.** The pip 0.8.2 wheel writes
   array-valued parameters into `overrides.yaml` as quoted strings and dict-valued ones
   as their Python repr, so `autoware_pose_initializer_node` dies with *"parameter
   {output_pose_covariance} is of type {double_array}, setting it to {string} is not
   allowed"* and, once that is fixed, *"Statically typed parameter
   'user_defined_initial_pose.enable' must be initialized"*. `shape_estimation` dies the
   same way. Fixed upstream in play_launch `f78745d` (now on main); build from source —
   a released wheel older than that is unusable here.
2. **Map projector.** The TUM pack on the NAS declares `projector_type: local`, which
   SSv2's `lanelet_loader` rejects outright (*"Unsupported projector type: local.
   Supported types are TransverseMercator and MGRS"*). `scripts/download_maps.sh` used
   to rewrite only `LocalCartesianUTM`; it now rewrites anything that is not
   TransverseMercator or MGRS.
3. **`just run` needs `CSB_CONFIG_DIR`.** Without it the bridge reads `./config`, which
   exists (the DDS xml lives there) but holds no `bridge_config.yaml` — so the run comes
   up with *no background AVs* and looks perfectly healthy. The recipe now exports the
   package config dir, and a missing config is a warning rather than an info line.
4. **Background AV poses were off the routing graph.** Three separate ways, all of which
   surface only as the pilot's *"The planned route is empty"*: the goal was past the end
   of its lanelet; the replacement lane (y=-53.7) carries **no subtype tag** — 65 of this
   map's 300 lanelets do not, and Autoware routes over `subtype=road` only; and the lane
   that is tagged runs **east to west**, so a vehicle facing east has its goal behind it.
   Working poses: spawn (140, -55.5) yaw 180, goal (110, -55.5) yaw 180, both on lanelet
   16946. Verify candidates with lanelet2 (`lanelet2.io.load` + `RoutingGraph`), never by
   eye off the OSM — the left/right boundaries are not stored in a consistent order, so a
   hand-built centerline can come out reversed.
5. **CARLA now boots on the scenario's town**, which costs nothing and saves 4.1 GB of
   peak RSS and 3.9 GB of VRAM. See `docs/design/carla-server-tuning.md` for the
   measurements and for what does not work (`-nullrhi` segfaults; the startup map cannot
   be set on the command line at all).

### Fresh-machine bring-up notes

- `src/color_names` is **not** a submodule and is not packaged: clone it by hand or the
  SSv2 build dies on `color_names/color_names.hpp`.
  `git clone --depth 1 https://github.com/OUXT-Polaris/color_names.git src/color_names`
- `pip install xmlschema`, or `scenario_test_runner` exits 1 on import.
- If `traffic_simulator` still fails to find a dependency's headers after that
  dependency builds, delete `build/traffic_simulator` — its CMake cache remembers the
  failed configure.
- CARLA lives at `~/Downloads/CARLA_0.9.16`. `AdditionalMaps` is deliberately **not**
  extracted; the base package already covers every town in the map pack.
- Debug loop for anything in a background AV's domain: `scripts/carla_bench.py` is for
  the server, and the world only ticks during a scenario — to work on the pilot without
  SSv2, spawn the vehicle and drive `world.tick()` from a plain CARLA client.

### The CARLA segfault is understood, and half-fixed

Three of eight runs died to a server `SIGSEGV` 24-40 s in. Symbolized (the `.debug` file
ships beside the packaged binary): `AInertialMeasurementUnit::ComputeGyroscope()`
dereferencing an owner that no longer exists, its `check(GetOwner() != nullptr)` having
been compiled out of the Shipping build. Destroying a vehicle leaves its sensors alive
and ticking.

csb now destroys a vehicle's attached sensors before the vehicle, on despawn and in
teardown; the real guard is in the fork on branch `sensor-owner-guards` (`e27ed518`) and
needs a **server rebuild** to take effect. Two consecutive runs afterwards left the
server untouched where the same sequence used to kill it.

### A reused ego stack cannot take a second scenario either

Same shape as the background AV, now measured on the ego: run A passes, run B on the
same stack ends in `AutowareError: waited for WAITING_FOR_ENGAGE ... current state is
PLANNING`. The mission planner first rejects the route from run A's *final* pose, then
accepts one 5 s later from the true spawn pose — and planning produces nothing after
that, `scenario_selector` falling silent from the end of run A onward. SSv2 restarts sim
time at ~0 every run.

**So: restart `just ego-av` as well as `just bg-av` between scenario runs.** Everything
csb owns survives a second run — teardown, respawn, reconnect — see
`docs/roadmap/007-repeatable-runs.md`.

### ROS domains: 1 for the ego, 2+ for background AVs, 0 unused

Domain 0 is where every unconfigured ROS process on the host lands, including other
people's demos, so a run whose ego lives there can be joined uninvited. `just ego-av`
and `just scenario` now export `ego_domain` (default 1) and `just bg-av` defaults to
domain 2; a second background AV takes 3. This bit for real on 2026-08-10, when someone
else's `planning_simulator` was already sitting in the domain a background stack wanted.

### A background stack is single-use per scenario run

Verified 2026-08-10, late session. On a **freshly started** background stack the pilot
does the whole job: localizes at the spawn pose, `Route set successfully (attempt 1)`,
engages, drives (`op_mode=2`). On a stack that has **already run a scenario** it used to
fail every time with "The planned route is empty", for a reason that had nothing to do
with the goal:

- `/api/localization/initialization_state` is latched, so after the first run it still
  reads INITIALIZED — for a vehicle that was destroyed at teardown. The pilot skipped
  its wait and routed from a pose belonging to nothing. Nothing re-triggered the
  automatic initializer either: it fires on the UNINITIALIZED → INITIALIZED edge, and
  the state never left INITIALIZED.
- acb `auto_drive` now detects that case, calls `/api/localization/initialize` (GNSS),
  and refuses to proceed until a *fresh* `/localization/kinematic_state` arrives.

With that in place the second run no longer routes off a phantom pose — but it still
does not drive, because the fresh pose never comes: SSv2 restarts sim time at ~0 for
every scenario, and a stack that has already seen a later clock stalls on the backward
jump. **So: restart the background AV stack before each scenario run.** `just bg-av`
takes ~4 minutes; budget for it.

### Still open

- Background AV **arrival** (see above) — needs sim-time budget, not a fix.
- The CARLA sensor-teardown crash is alive and well on this host: five systemd restarts
  during rapid spawn/teardown cycling, `ERROR: Invalid session: no stream available`.
  Still the highest-value fix in the jerry73204/carla fork.
- Occasional `change_to_stop` unavailability on a stack that has been up across several
  runs (one failure in six). A rerun cleared it.

## Where things stand

- **E2E green and reproducible**: `town01_ego_drive.xosc` has passed three times
  on fresh stacks (SSv2 `Passed`, JUnit clean, ~39 s spawn-to-goal). Working
  recipe: CARLA (systemd unit, offscreen) → `carla_scenario_bridge`
  (`CSB_CONFIG_DIR=src/carla_scenario_bridge/config`) → `just ego-av` → wait for
  play_launch "Startup complete" + `/api/operation_mode/change_to_stop` in the
  service list → `just scenario ...`. Verdict: interpreter err log +
  /tmp/scenario_test_runner/result.junit.xml.
- **011 (acb re-attach) done**: acb detects ego despawn, releases sensors,
  re-attaches to the next spawn — across map reloads and server replacements.
  Fresh ego stack per scenario is no longer required.
- **007 half-verified**: run A reproduces; run B on a reused stack has never
  received a verdict — every attempt was cut down by CARLA OOM-kills from the
  shared GPU's training jobs (ten in one day). All pieces are committed; it
  needs one quiet ~15 min window.
- **010 mostly verified**: two-domain run works — D0 ego + D1 background stack
  coexist (after the DomainGain fix below), the bridge spawns `bg_av_1`, its
  acb attaches and GNSS-initializes localization, and the ego scenario passed
  with the background AV in-world. Unverified: the pilot's engage leg (the bg
  AV physically driving) — same GPU story.

## Pinned fixes worth knowing about

- Two `/clock` publishers (double acb_bridge include) was the great
  availability killer — `launch_vehicle_interface:=false` in ego_av/
  background_av launch files. Never run two clock publishers in one domain.
- CycloneDDS `DomainGain 1000` (config/cyclonedds-localhost.xml): default 250
  overlaps domain 0's port range with domain 1's base once
  MaxAutoParticipantIndex is 300 → "Failed to find a free participant index".
- Bridge smooths reported acceleration over 3 frames; xosc Performance bounds
  15/15. PhysX one-frame contact jolts are otherwise scenario-fatal.
- play_launch: lost-load rescue landed upstream; `just ego-av` passes
  `--load-node-timeout 120 --load-total-budget 180`.
- acb rebuilds MUST set `CARLA_VERSION=0.9.16` or the 0.10.0 prebuilt links in
  and crashes with `std::bad_array_new_length` at connect.

## Operational traps (hard-won, avoid relearning)

- Failed scenario runs leave interpreter zombies that hijack the next run's
  lifecycle services and desync the bridge's ZMQ REP socket. Kill scenario
  processes by comm name, then restart the bridge process (it is stateless).
- Stale per-domain `ros2 daemon`s lie; use `--no-daemon` when checking D1.
- The world only ticks during a scenario (SSv2 owns the tick) — a background
  AV cannot drive between runs.
- `pkill -f` with any pattern that appears in your own wrapper's command line
  kills the wrapper. Match on comm, not args.

## Next actions

1. Quiet GPU (`nvidia-smi` < ~2 GB): 007-B verdict, then the bg-AV-drives leg
   of 010. Both are pure reruns.
2. CARLA sensor-teardown crash fix in the jerry73204/carla fork — ends the
   OOM/teardown crash class that has eaten two verification sessions.
3. Upstream batch: play_launch rescue, SSv2 `carla-compat` arrived_goal patch.

## Repo pins at checkpoint

- csb main `3b20205` (+ this checkpoint)
- acb main `aeb2031`
- SSv2 fork branch `carla-compat` `f77cd12c3`
- play_launch main `9d06d08`
- carla-rust master `2718365`, CARLA fork branch
  `worker-thread-exception-containment`, release `carla-rust/0.9.16-3`
