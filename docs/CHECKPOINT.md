# Session checkpoint — 2026-08-12

State of the SSv2+CARLA integration after the exitSuccess campaign. For the full
per-phase anatomy see `docs/roadmap/012-ssv2-unmanaged-autoware.md`; this file is
the cross-machine handoff summary.

## Update — 2026-08-10, second machine (newslab-server243, RTX 5090)

The stack was brought up from a clean clone on a second host and the two-domain run
now goes further than it ever has:

- **Ego E2E green here too**: `town01_ego_drive.xosc` passed five consecutive times
  (SSv2 JUnit clean), spawn-to-goal ~35-50 s.
- **The background AV drives to its goal** (2026-08-12): `Localization: INITIALIZED at
  (139.95, -55.48)`, `Route set successfully (attempt 1)`, `Autonomous mode engaged`,
  `ARRIVED at goal after 18.8s`, with the ego passing its own scenario in the same run.
  It needed sim time, nothing else: the pilot's cold start is ~62 s from spawn to
  arrival and the world only ticks during a scenario, so use
  `scenarios/town01_two_av.xosc` (ego runs 212 m, timeout 300 s) rather than
  `town01_ego_drive.xosc` (~50 s, cuts the pilot off just after engage) whenever a
  background AV has to finish.
- **The ego follows it** (2026-08-15): with `bg_av_1` in the ego's own lane the ego closes
  to 18 m, slows, and holds a ~24 m gap for the rest of the run, still passing its
  scenario. 010 is complete; see the section below for what that measurement cost.
- **The ego stops for a pedestrian** (2026-08-16): on
  `scenarios/town01_pedestrian.xosc` the ego halts 9 m short of a walker crossing its
  lane, holds 53 s, and resumes when the walker reaches the far walkway. A street barrier
  3 m off the driving line is detected too. Both come back classified **UNKNOWN** —
  detection works, classification is the camera leg 009 has parked. 008's acceptance is
  closed.
- **Teardown is clean**: after each run CARLA holds no vehicles or sensors, background
  AV included (010's "no background AV left behind"). A background AV is not an SSv2
  entity, so it outlives the scenario that spawned it *by design* — it is destroyed at the
  next `Initialize` or at bridge shutdown, both verified, with its sensors, via
  `destroy_with_children`. Pedestrians and props leak nothing either: 0 vehicles,
  0 sensors, 0 walkers, 0 non-map props after every run.

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

### Never destroy a vehicle while its sensors are attached

**The rule**: destroy an actor's children first, always, in every language and every
client. In Rust that is `ActorBase::destroy_with_children()` (carla-rust `9596cde`); in
Python, sweep `world.get_actors().filter('sensor.*')` for `sensor.parent.id == vehicle.id`
before `vehicle.destroy()` — see `demo_scenario.py::destroy_attached_sensors`. Do not
write a fresh sweep by hand; the helper exists so there is one implementation to get
right.

**Why it matters**: three of eight runs once died to a server `SIGSEGV` 24-40 s in.
Symbolized (the `.debug` file ships beside the packaged binary):
`AInertialMeasurementUnit::ComputeGyroscope()` dereferencing an owner that no longer
exists, its `check(GetOwner() != nullptr)` compiled out of the Shipping build. CARLA does
not take a vehicle's sensors down with it — they stay alive, parentless and still
ticking. Cameras give a second crash on the render thread from the same orphaning
(`BeginReleaseResource`). Upstream has this reported and unfixed:
carla-simulator/carla#5812, #7046, #3197, #7987.

**Why the helper reads the server's actor list** rather than anything local: the client
that owns a vehicle's lifetime is usually not the one that attached its sensors. csb
spawns and despawns vehicles; acb attaches sensors to them. acb does clean up after
itself, but only once it notices the vehicle is gone, and the crash lives in that window.
Only the destroyer can close it. This is a deliberate, documented bend of invariant 2.

**Result**: the server has now been up 35 hours across dozens of runs with
`NRestarts` unchanged, against 5 restarts in the 11 hours before the ordering was fixed.

**The fork patch is not needed for this workflow.** `jerry73204/carla` branch
`sensor-owner-guards` (`e27ed518`) guards the IMU at the source, but it is UE4 plugin
code: it needs a full server rebuild and a forked binary to distribute, and it would not
have covered the render-thread crash anyway. It stays parked as an upstream PR candidate.
Reach for it only if something outside our control starts orphaning sensors.

### `just ego-av` leaked its API adaptors, and a stack eventually refuses to start

The recipe backgrounded two `ros2 launch` API adaptors and then `exec`ed play_launch,
replacing the shell — so no trap could fire and every restart left another pair running.
Sixteen accumulated in one session, the oldest 11.5 hours old, all holding the same
ADAPI node names. A fresh stack then stalls at **92/93 composables** with
`/adapi/node/autoware_state` pending forever (`LoadNode service call timed out after
120s ... deferring to ComponentEvent`), which play_launch's lost-load rescue does not
recover. Killing the strays and restarting brought the same launch up 93/93 first try.

Fixed: the recipe keeps the adaptor pids, drops the `exec`, and traps `EXIT INT TERM` to
kill their process groups. If a stack ever stalls short of 93/93 again, count them:
`ps -eo args | grep -c internal_api_adaptor.launch.py` should be 2, not 16.

### Stacks are reusable across runs (2026-08-12) — acb was never noticing the despawn

**Two scenario runs back to back on one stack now both pass**, with nothing restarted
between them: same bridge process, same ego Autoware, same background stack, same CARLA
server, `NRestarts` unchanged. 50 s each.

Getting there meant chasing a failure that pointed everywhere except at its cause. Run B
used to end in `AutowareError: waited for WAITING_FOR_ENGAGE ... current state is
PLANNING`, and the ego's diagnostic graph blamed localization (scan_matching_status,
accuracy, sensor_fusion_status), perception (`topic_rate_check/pointcloud`) and planning
(`topic_rate_check/trajectory`) all at once, which is what `autonomous=False` on
`/system/operation_mode/availability` is made of. Counting messages per hop showed
behavior planning still producing `path_with_lane_id` at 43 points while
`lane_driving/trajectory` and `scenario_planning/trajectory` sat at zero.

The cause was one line in acb: `Actor::IsAlive()` is a **client-side flag**, true until
*that* client destroys the actor. acb never destroys the vehicle — csb does, from its own
client — so acb's session never ended, and it went on publishing sensor topics whose
CARLA actors were gone. Autoware saw publishers with no data behind them. Fixed in acb
`ab5dc67` by asking the world snapshot, which is the server's actor list for the frame
just ticked. (`World::GetActor` is not a substitute: tried first, never fired, same
client-side registry problem.)

Both bridges now log `Vehicle 'hero' (actor N) is gone from the world` within seconds of
the next run's first tick and re-attach. Across the four runs before the fix, neither
ever noticed.

### The ego follows the background AV in its own lane (2026-08-15)

[010]'s last criterion is closed: the ego's Autoware perceives the background AV and
*follows* it. Both on lanelet 6583 — `bg_av_1` from x=230 to x=110, the ego from x=320 to
x=135 behind it, 25 m of clearance past the ego's goal so the parked background AV never
blocks the lane the ego still has to finish in.

```
t+29s truth_x=286.26 est_x=286.03 pose_err=0.23 gap_to_bg= 56.3 speed=3.9 objects=7
t+39s truth_x=248.09 est_x=247.84 pose_err=0.25 gap_to_bg= 18.1 speed=2.9 objects=4
t+60s truth_x=195.95 est_x=195.57 pose_err=0.39 gap_to_bg= 23.7 speed=3.7 objects=3
t+70s truth_x=157.12 est_x=156.78 pose_err=0.34 gap_to_bg= 24.4 speed=3.9 objects=3
```

Closes to 18 m, slows 3.9 -> 2.5 m/s, then holds ~24 m and matches speed — and still
passes its scenario. `scripts/lead_vehicle_probe.py` produced that; it samples CARLA's
ground truth and Autoware's estimate together, which is the only way to tell "the ego
stopped" from "the ego thinks it is somewhere else".

Three traps this cost a day to find, all worth knowing before the next multi-AV run:

1. **Never trust "the nearest tracked object".** A ghost object trails ~1 m off the ego's
   own bumper for the entire run, so the nearest object is always the ego itself. The
   question is the nearest object *ahead, in lane, and outside a 5 m self-radius*, which
   is what `scripts/perception_probe.py` now reports as `BEST LANE HIT`.
2. **A fresh CARLA client sees zero actors until it has ticked.** In synchronous mode
   `get_actors()` on a just-connected client returns an empty list whether the world is
   empty or full — indistinguishable from a clean teardown. `world.wait_for_tick()` first.
3. **A harness that says "restart" must actually kill.** `scripts/two_av_run.sh` only ever
   started a background stack, so its second invocation ran two full Autoware stacks in
   domain 2 against one vehicle. The ego's LiDAR fell to 1.1 Hz, its tracked-object count
   went from 4 to 50 as buildings stopped being subtracted from the pointcloud map, and
   its trajectory dropped to a 0.25 m/s crawl. Nothing published a planning velocity
   factor or a virtual wall, so the planner-side evidence was all absent — the diagnostic
   that separates this in seconds is `ros2 topic hz` on the ego's LiDAR: 10 Hz healthy,
   ~1 Hz starved. Load average does not separate them, because a stalled run keeps both
   stacks alive five times longer and the load is mostly an effect.

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

With that in place the second run no longer routes off a phantom pose. At the time it
still did not drive, and this file blamed sim time restarting at ~0 — but the real reason
was the same acb bug described above: the background AV's sensors were gone too, so no
fresh pose could ever arrive. That is fixed (acb `ab5dc67`), and `bg_av_1`'s bridge is
now seen re-attaching on the second run alongside the ego's.

**Untested as of 2026-08-12**: whether the background AV's *pilot* completes route and
engage on a reused stack. The pilot exits after its first run, so a second run needs it
restarted — `just bg-av`, or run `acb_pilot auto_drive` on its own against the stack that
is already up. Nothing is known to require restarting the stack itself any more.

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
- Destroying a vehicle with sensors still attached segfaults the server. Use
  `destroy_with_children()` in Rust, the sensor sweep in Python — see the rule above.
  This applies to throwaway scripts too; a sandbox script that skipped it cost a run.
- A CARLA client's `Actor::IsAlive()` and `World::GetActor()` can both answer from the
  client's own registry, so neither notices an actor another client destroyed. Check
  `world.snapshot().contains(id)` instead — that is the server's list for the ticked
  frame.
- A freshly connected CARLA client's `get_actors()` is empty until it has seen a tick.
  In synchronous mode that makes a full world look like a torn-down one. `wait_for_tick()`
  before believing an actor count, or keep one client for the whole observation.
- The map pack's traffic lights are untyped: `subtype=""` on the regulatory element and
  `type=""` on the light way, where both should read `traffic_light`. SSv2 then rejects
  the id (*"neither a traffic light ID not a traffic relation ID"*) and Autoware finds no
  signals at all. `scripts/repair_lanelet_traffic_lights.py` fixes it and
  `download_maps.sh` runs it; re-run it on any new pack.
- A camera namespace with no `<ns>_traffic_light_map_based_detector.param.yaml` makes
  that node exit at startup and the launch carry on. Symptom: perception publishes empty
  signal arrays forever and the ego stops at every stop line for want of information.
  Only `camera6` and `camera7` have files, which is why acb's traffic-light camera is
  named camera6.
- **Autoware's `data_path` must be writable.** TensorRT builds each engine from the ONNX
  and writes it next to that file; pointed at the packaged root-owned directory the build
  succeeds, the write fails with `[E] [TRT] Fail to open engine file`, the component
  constructor throws, and the node silently never loads. Run acb's
  `scripts/link_autoware_data.sh` to mirror the packaged tree into `$HOME/autoware_data`.
  Engines then cache instead of costing 33 s per model per launch.
- Camera topics carry the **optical** frame (`<ns>/camera_optical_link`), not the mounting
  frame. Anything projecting 3D into the image reads that frame off the header and assumes
  z forward; stamped with `camera_link` every projection lands behind the image plane.
- Perception keeps publishing the *previous* run's object list for the first seconds
  after `Initialize`. A leftover object sitting on a new entity's coordinates reads as a
  detection 200 m away — reject any match beyond sensor range before believing it.
- Town01 holds ~150 `static.prop.mesh` actors of its own. When looking for props a
  scenario spawned, filter that type out or the two you care about vanish into 150 lines.
- An ego crawling at 0.25 m/s with tens of phantom obstacles is usually starved sensors,
  not a planner decision. `ros2 topic hz` on the ego's LiDAR settles it: 10 Hz healthy,
  ~1 Hz starved. No velocity factor and no virtual wall means nothing in planning asked
  for the stop, so do not go looking for a stop reason.

## Next actions

1. **Upstream batch**: play_launch rescue and compound-parameter fix (both landed on
   its main), SSv2 `carla-compat` arrived_goal patch, carla-fork exception containment
   and the IMU owner guard (`sensor-owner-guards`).
2. **A second background AV** (domain 3). Two stacks cost ~48 load and leave the ego's
   LiDAR at 10 Hz on this host; a third is unmeasured, and the failure mode when the
   host runs out is a quiet crawl, not an error.
3. **Background AV on a reused stack** — the pilot exits after its first arrival, so a
   second scenario needs the background stack restarted (`scripts/two_av_run.sh` does
   this). Making the pilot re-arm on a new run would remove the restart.

## Repo pins at checkpoint

- csb main `05af09b` (+ this checkpoint)
- acb main `ece42e7`
- SSv2 fork branch `carla-compat` `f77cd12c3`
- play_launch main `9d06d08`
- carla-rust master `9596cde` (adds `destroy_with_children`), CARLA fork branch
  `worker-thread-exception-containment`, release `carla-rust/0.9.16-3`
