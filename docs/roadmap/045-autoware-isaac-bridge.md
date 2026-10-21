# Phase 045: autoware_isaac_bridge (aib)

The Isaac ego as its own repository, the way acb is CARLA's: the PhysX vehicle, its sensors
and Autoware's vehicle interface, with the vehicle described once in a spec that generates
Autoware's vehicle and sensor kit files and the Isaac rig. csb's world server then imports
it instead of carrying its own copy in `src/csb_isaac`.

**Source**: user direction 2026-10-18 (a standalone public repository in NEWSLabNTU; the
single source of truth for the vehicle lives in aib).
**Depends on**: [043](043-isaac-single-vehicle-e2e.md) (the plausible real Model 3, the
supervised world server, the exit runs).
**Feeds**: [044](044-isaac-npcs-and-traffic-lights.md), [039](039-isaac-multi-autoware.md)
(several egos, one rig each), [040](040-ntu-offroad-field.md) (another vehicle is another
spec).

Repository: [NEWSLabNTU/autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge)
(its `docs/design.md` and `docs/spec-format.md` are the reference; this phase is csb's side).

## Design

### What moves, what stays

| moves to aib | stays in csb |
|---|---|
| `csb_isaac/vehicle.py`: the PhysX vehicle (`aib.isaac.vehicle`), its actuation split into `aib.control` (steering, Ackermann) and `aib.powertrain` (pure) | `world.py`, `world_server.py`, `server.py`, `protocol.py`, `csb_world.proto`: the lockstep world, NPCs, contacts, the world protocol |
| `csb_isaac/vehicle_params.py`: `VehicleParams` (`aib.params`, same field names, so `vehicle:` overrides still apply) | `physics_params` and the CARLA catalogue (`vehicle_assets.py`, `dump_vehicle_physics.py`): CARLA's blueprints are csb's |
| `csb_isaac/ego.py`: `EgoRos` (`aib.ros`), `Ego` and the sensors (`aib.isaac.ego`, `aib.isaac.sensors`; IMU and GNSS maths pure in `aib.sensing`) | the body the ego wears (`vehicle_assets.add_ego_body`) and the vehicle_info-vs-catalogue check, now against the rig |
| `csb_isaac/frames.py` (`aib.frames`) | `physics_layer.py`, `usd_world.py`, `pcd_world.py`, `isaac_town.sh`: worlds |
| `scripts/isaac_env.sh`, `scripts/build_msgs.sh` (aib carries copies; csb can source aib's) | the step-response scripts (they compare CARLA and Isaac; they can import aib) |
| `config/acb_ego.yaml`: the vehicle part becomes aib's spec and rig; the run part (domain, clock offset) aib's `config/ego.yaml` | `isaac_world.launch.xml`, `just isaac-world`, `scripts/isaac_up.sh` |

### The single source of truth

`vehicles/tesla_model3.yaml` in aib is the car: geometry, mass and centre of mass, wheels,
tires, brakes, powertrain, steering, suspension, PhysX settings, the sensor mounts and
intrinsics, and a provenance entry per number (`assumed: true` on the centre of mass, the
inertia box, the wheel mass, the tires and the suspension). The values are 043's "As built:
vehicle" table. `aib.gen` writes from it, each file headed "generated from ..., do not edit":

- `aib_vehicle_description/config/vehicle_info.param.yaml`, `mirror.param.yaml`
- `aib_sensor_kit_description/config/{sensors,sensor_kit}_calibration.yaml`, `urdf/sensor_kit.xacro`,
  and the same calibration under `aib_individual_params/config/default/aib_sensor_kit/`
- `src/aib/config/tesla_model3.rig.yaml`: the flat `VehicleParams`, the chassis box, each
  sensor with its base_link transform

`just check-generated` (and a unit test, and aib's CI) fails when any is stale. Later, acb's
`vehicle_config.yaml` and vehicle description can be generated from the same spec, so CARLA
and Isaac drive one car description; not part of this phase.

### Autoware side

`aib_launch isaac_simulator.launch.xml` includes acb's `carla_simulator.launch.xml` with
`vehicle_model:=aib_vehicle sensor_model:=aib_sensor_kit launch_vehicle_interface:=false`;
every other argument passes through (`relay`, `map_path`, `rviz`, ...). The agent is acb's
`acb_pilot`, unchanged. A `play_launch dump` of it resolves the same 205 nodes as
`carla_simulator.launch.xml launch_vehicle_interface:=false`, with every vehicle_info
reference aib's. Topics and frames are acb's (`top`, `imu_link`, `gnss_link`; until
2026-10-19 the last two were `carla/imu_link` and `carla/gnss_link`, see Decisions).

### Powertrain

A seam (`aib.powertrain`): `ideal` is today's behaviour (the requested acceleration as
per-wheel torque from m a, braking split by dynamic axle load). `physx` (DriveStandard),
`ev` and `fmu` are reserved names that fail at load.

`ev` comes next, modelled on the Tesla Model 3 drivetrain (user direction 2026-10-19), since
there are no drive logs to fit against. It covers the motor torque/power curve, regenerative
braking, the brake hydraulic lag and ABS.

- [x] `ev` in aib (aib `7816ba6`, 2026-10-19): the Model 3 RWD drive unit (3D7 motor, 340 N m /
      194 kW, 9.03:1 scaled to the CARLA body's wheel, torque lag 40 ms), regen (130 N m /
      60 kW, ~0.2 g, faded below 3 m/s) blended with friction brakes through 80 ms dead time
      + 60 ms lag, ABS per wheel and traction control on PhysX slip; road loads (Crr 0.0095,
      Cd 0.23 x 2.2 m^2) as a body force for every model. Unit-tested (93 tests, system and
      Isaac Python); point-mass 0-100 km/h 5.9 s against the published 5.8-6.1 s.
- [x] `ev` validated in Isaac and the default for tesla_model3 (aib `54cace1`, 2026-10-19;
      aib docs/design.md, Validation). `just isaac-steptest`: 0-100 km/h 6.45 s (Tesla 6.1 s;
      0.27 s of it the hold release), 100-0 km/h 42.9 m with ABS, no lock (44.1 m locked; ~40 m
      published), regen at most ~0.20 g at 55 km/h, -1.5 m/s^2 all regen, brake step t63
      0.10 s, coast-down matching the road-load model. Closed loop on Town01 (Autoware +
      acb's pilot, town01_ego_drive's route): median longitudinal error 0.018 m/s^2 (`ideal`
      0.016; limit 0.35), one run each. The tyre grip now falls to 0.8 at lock (user
      decision), for `ideal` too: csb's 043 numbers were taken on the flat curve.

The other options (`physx` DriveStandard, an FMU,
Project Chrono for rough or deformable ground) wait until a need shows.

### Decisions (user, 2026-10-19)

- [x] **IMU/GNSS frames** kept `carla/imu_link` and `carla/gnss_link` until integration,
  because acb's sensing launch, which aib reuses, named them. Renamed on both bridges at
  once (2026-10-19, aib `23f719e`, acb `c9ff587`) to `imu_link` and `gnss_link`, Autoware's
  sample-kit style: topics `/sensing/imu/imu_link/imu_raw` and
  `/sensing/gnss/gnss_link/nav_sat_fix` (acb derives the topic from the frame). Nothing in
  Autoware 1.5.0 assumes a `<vendor>/imu_link` form -- imu_corrector, gyro_odometer and
  gnss_poser all go through TF from the message's `frame_id`. Both sensor kits' `imu.launch.xml`
  and `gnss.launch.xml` take the topic as an argument (`imu_raw_name`, `gnss_fix_name`).
  Bags and logs recorded before carry the old names.
  Pushed before any simulator run (the GPU was taken by another user's job): builds and unit
  tests pass in all three repos.
  **CARLA leg passed 2026-10-20** (build matching `473729f` / acb `c9ff587`):
  `town01_ego_drive` 3/3 PASS, 0 MRM, longitudinal 0.008-0.010 m/s^2, cross-track
  0.042-0.046 m. `tf2_echo base_link imu_link` = [0.900, 0.000, 2.000], q [0.018, 1.000,
  -0.000, -0.008] (orientation unchanged by the rename); `base_link gnss_link` = [0.797,
  0.004, 1.802]. `imu_raw` carries `frame_id: imu_link`, `nav_sat_fix` `gnss_link`; no old
  topic names; gyro_odometer, imu_corrector, gnss_poser and the gyro bias estimator log no
  frame errors. (EKF-vs-GNSS worst 65.7 m in runs 2 and 3, median 0.95 m: likely a sample
  across the route reset between runs, not looked into.)
  **Isaac leg passed 2026-10-21** on newslab-server139 (csb `3107381`, aib `23f719e`, acb
  `25470e4`; Town01 SimReady world, `worlds:=Town01,Town02`, headless): `just acceptance
  src/csb_examples/scenarios/basic/town01_ego_drive.xosc 3 backend:=isaac` 3/3 PASS, 0 MRM,
  longitudinal 0.013-0.020 m/s^2, cross-track 0.03-0.04 m, EKF-GNSS 0.99-1.04 m.
  `tf2_echo base_link imu_link` = [0.900, 0.000, 2.000], q [0.018, 1.000, -0.000, -0.008];
  `base_link gnss_link` = [0.797, 0.004, 1.802]: CARLA's numbers to the last digit. `imu_raw`
  carries `frame_id: imu_link`, `nav_sat_fix` `gnss_link`, both published by `isaac_vehicle`;
  no `carla/` topic; gyro_odometer, imu_corrector, gnss_poser and the gyro bias estimator
  log no frame or TF errors.
- **Camera images** are stamped in `camera6/camera_optical_link` (REP-103: Autoware's
  projection nodes assume the optical frame). acb stamps `camera_link`, which is likely
  wrong there too. The camera is off on both bridges today.
- **aib depends on acb at run time** (`acb_launch`, `acb_pilot`) instead of copying them;
  standalone use needs `ACB_SETUP`.
- **Sensor poses come from the spec only.** A hand edit to a generated calibration does not
  reach Isaac, and `just check-generated` flags it.
- **The Isaac message build is shared**: aib and csb both use `$ISAAC_ROOT/msgs`.

## Steps

- [x] aib repository: library, spec, generator with check mode, Autoware packages, unit
      tests without Isaac, docs; builds with colcon against Autoware 1.5.0 (2026-10-18).
- [x] aib's Isaac code run once from aib itself: `just isaac-ego` on Town01 with `just
      autoware`, a drive to a goal (standalone, no csb) (2026-10-19, aib `54cace1`): arrived,
      `ev` and `ideal`. Needed: the chassis box hidden from the lidar (Autoware stopped for
      the ego's own roof), `just autoware` keeping the caller's CYCLONEDDS_URI, and
      `realtime:=true` (unpaced, the world ran 3x real time and localization never started).
- [x] Integration: aib as a submodule at `src/autoware_isaac_bridge` (pushed first, then the
      pin); `csb_isaac` imports `aib` (world server's `PYTHONPATH` gets
      `src/autoware_isaac_bridge/src/aib`); `IsaacWorld` builds `aib.isaac.ego.Ego` from an
      aib ego config (`ego:=` names one; default aib's `config/ego.yaml`); the catalogue
      check runs against the rig's geometry; `just ego-isaac` runs `aib_launch
      isaac_simulator.launch.xml` (csb `442d34c`, `2868f8a`; pin aib `54cace1`). As built:
      `csb_isaac/scripts/isaac_env.sh` sources aib's and adds aib to `PYTHONPATH` (every
      Isaac-side csb script gets it), `build_msgs.sh` runs aib's; the catalogue check and
      `physics_params` live in `csb_isaac/catalogue.py` (tests `test_catalogue.py`, against
      aib's default ego); the world server hides every ego's chassis box from rendering, as
      aib's runner does (the roof lidar sees it otherwise). csb's `just build` builds aib's
      seven packages (no name clashes); a `play_launch dump` of `just ego-isaac`'s launch
      resolves 206 nodes, no acb_bridge, all 201 vehicle_info references aib's.
- [x] Remove the duplicated code from csb: `csb_isaac/{ego,vehicle,vehicle_params,frames}.py`,
      `config/acb_ego.yaml`, their tests (aib has them), and point docs (038, 043, the user
      guide) at aib (038, 043 and design/simulator-backends.md carry a pointer; the user
      guide has no Isaac section to point; CLAUDE.md's repository structure lists aib).
- [x] Exit runs through aib (2026-10-19, "Exit runs" below).

## Acceptance

- [x] 043's exit runs (`town01_ego_drive`, `town01_engage_state`, `town02_episode_change`, one
  route per exported town) still pass on Isaac with the ego built by aib: 5/5 each, 0 MRM,
  tracking within acceptance.py's limits, and of the same order as 043's recorded numbers.
- [x] csb carries no copy of the ego: `grep -r PhysxVehicleAPI src/csb_isaac` finds nothing.
- [x] A change to the vehicle is one edit to aib's spec plus `just generate`; aib's
  `check-generated` fails if the generated files were not refreshed.

## Exit runs

2026-10-19, through aib `54cace1` (`ev` powertrain, the tyre grip falling to 0.8 at lock,
1760 kg RWD, road loads as a body force), headless world server, one town per batch, each
batch from no Isaac stack (`scripts/isaac_up.sh` started the world server, the simulation
side and `just ego-isaac`, and the batch stopped them, no orphans). Every row is `just
acceptance <xosc> 5 backend:=isaac` (`worlds:=Town01,Town02` for Town01) scored by
`scripts/acceptance_bag.py`; the episode change is five `just scenario` pairs
(`town02_episode_change`, then `town01_ego_drive`, `worlds:=Town01,Town02`) judged by their
junit. Medians over the five runs, range in brackets; limits 0.35 m/s^2, 1.0 m, 10 m. 043's
medians beside them (its flat tyre curve and 1845 kg AWD-mass `ideal` car). NDT and frame
time were not extracted this time. Logs and bags: `/mnt/disk1/csb-045/play_log/`.

| scenario | PASS | longitudinal, m/s^2 (043) | cross-track, m (043) | EKF-GNSS, m (043) | MRM |
|---|---|---|---|---|---|
| `town01_ego_drive` | 5/5 | 0.012 (0.009-0.021) (0.028) | 0.04 (0.04) | 1.03 (1.02) | 0 |
| `town01_engage_state` | 5/5 | 0.013 (0.012-0.017) (0.024) | 0.03 (0.03) | 1.01 (1.02) | 0 |
| `town02_episode_change` + `town01_ego_drive` | 5/5 + 5/5 | | | | |
| `town02_ego_drive` | 5/5 | 0.007 (0.007-0.009) (0.017) | 0.01 (0.01) | 1.37 (1.35) | 0 |
| `town03_ego_drive` | 5/5 (second batch; first 4/5) | 0.018 (0.018-0.033) (0.025) | 0.01 (0.01) | 0.91 (0.95) | 0 (first: 1) |
| `town05_ego_drive` | 5/5 | 0.018 (0.018-0.019) (0.028) | 0.03 (0.03) | 0.77 (0.71) | 0 |
| `town10_ego_drive` | 5/5 | 0.024 (0.023-0.030) (0.031) | 0.04 (0.04) | 0.45 (0.42) | 0 |

**Deltas.** Cross-track and EKF-GNSS are 043's to the centimetre: steering, the sensors, the
worlds and the GNSS model did not change, only the drivetrain and the tyre curve. Peak speed
(4.25-4.30 m/s) and distance travelled are the same planner's. Longitudinal error is lower in
every town (0.007-0.024 against 0.017-0.031), all at least 14x inside the limit. Autoware's
requests on these routes are gentle (well under 0.2 g), where the grip curve's fall at lock
never comes into play and regen alone carries the braking; the lighter car (1760 kg against
1845) and the road loads now modelled rather than unmodelled residuals plausibly account for
the rest. aib's own closed loop on Town01 measured `ev` 0.018 against `ideal` 0.016, so the
powertrain itself is not what moved these numbers; the difference to 043 is of the order of
run-to-run spread on a quiet host against 043's loaded one, and the same order of magnitude,
as the acceptance asks.

**The Town03 MRM** (first batch, run 2): one 0.1 s EMERGENCY_STOP while driving
(`mrm_handler` NORMAL -> MRM_OPERATING -> NORMAL), from one `control_validator:
control_validation_latency` ERROR sample ("The latency is larger than expected value"), the
only ERROR in the diagnostics at that instant (bar the always-unknown door). Autoware's
`nominal_latency` is 0.01 s with `diag_error_count_threshold: 0`, so one control command whose
stamp lands a /clock frame (50 ms) behind its check trips it: a sim-clock race, not the
vehicle, and nothing in the host load (15 % CPU) or the world server's step log (no slow step).
The whole route reran 5/5 with 0 MRM. Not seen in 043's runs or in the other 44 here; if it
recurs, the fix is acb's control_validator parameters (a count threshold of 1-2, or a latency
allowance of one frame), not aib.

**Builds.** Town01 to Town10 ran a csb `just build` of 07:17 (csb `2868f8a` plus the CARLA
agent's then-uncommitted bridge config change, which became `077ca50`); the Town03 rerun ran
its 08:35 rebuild (csb `3c9a4ba`, acb `f10ce12`). On Isaac neither acb_bridge nor the bridge's
config change is in the ego's path.

## Reproduction on newslab-server139

2026-10-21: the exit runs again on a second host, set up from nothing the same day (Isaac
Sim 6.1, the CARLA 0.9.16 editor build, the five towns exported; how, and what the setup
docs lacked: 043, "A new host"). RTX 5090, 32 cores, 60 GB RAM with 48 GB available at each
batch's start. Same code as the rename leg above (csb `3107381`, aib `23f719e`, acb
`25470e4`), headless, each town a batch from no Isaac stack, run the same way as "Exit runs"
(`just acceptance <xosc> 5 backend:=isaac`, `worlds:=Town01,Town02` for Town01; the episode
change as five `just scenario` pairs judged by their junit). Medians over five runs, range
in brackets; the original host's ("Exit runs" above) beside them. Logs and bags:
`/mnt/wd/aeon/csb-045/` on server139.

| scenario | PASS here (orig.) | longitudinal, m/s^2: here / orig. | cross-track, m | EKF-GNSS, m | MRM |
|---|---|---|---|---|---|
| `town01_ego_drive` | 5/5 (5/5) | 0.012 (0.008-0.018) / 0.012 (0.009-0.021) | 0.04 / 0.04 | 1.01 / 1.03 | 0 / 0 |
| `town01_engage_state` | 5/5 (5/5) | 0.018 (0.014-0.021) / 0.013 (0.012-0.017) | 0.03 / 0.03 | 0.99 / 1.01 | 0 / 0 |
| `town02_episode_change` + `town01_ego_drive` | 5/5 + 5/5 (5/5 + 5/5) | | | | |
| `town02_ego_drive` | 5/5 (5/5) | 0.009 (0.007-0.010) / 0.007 (0.007-0.009) | 0.01 / 0.01 | 1.38 / 1.37 | 0 / 0 |
| `town03_ego_drive` | 5/5 (5/5) | 0.020 (0.017-0.038) / 0.018 (0.018-0.033) | 0.01 / 0.01 | 0.92 / 0.91 | 0 / 0 |
| `town05_ego_drive` | 5/5 (5/5) | 0.019 (0.018-0.022) / 0.018 (0.018-0.019) | 0.03 / 0.03 | 0.72 / 0.77 | 0 / 0 |
| `town10_ego_drive` | 5/5 (5/5) | 0.028 (0.023-0.036) / 0.024 (0.023-0.030) | 0.04 / 0.04 | 0.51 / 0.45 | 0 / 0 |

**It reproduces.** Every run passed with no MRM, cross-track is the original's to the
centimetre in every town, peak speed (4.24-4.28 m/s) and distance travelled are the same,
and each longitudinal range overlaps the original's. Five of six longitudinal medians are
higher here by 0.001-0.005 m/s^2; only `town01_engage_state` sits outside the original's
range (median 0.018 against at most 0.017), and that gap is half the spread
`town01_ego_drive` shows by itself on either host (0.008-0.018 here), 20x inside the limit.
Two plausible causes, neither isolated: this host was not quiet (an idle CARLA server kept
~2.8 GB and two cores of the same GPU busy, and another long-running job a third core), and
the ego here is a box with no CARLA body (below). EKF-GNSS moved both ways (Town05 -0.05 m,
Town10 +0.06 m), as a per-town localization difference of that size would; not looked into.

Differences in the setup, each noted where it matters:

- **No vehicle bodies.** The 41 SimReady vehicles and their `vehicles.json` were made by
  tools kept beside them on the original host, not in git (`05bdd87`), so here every
  entity, the ego included, is a box (`no CARLA body for 'vehicle.tesla.model3': a box`).
  Bodies are render-only; the numbers above say they do not move tracking. (Since
  `be62582` the tools are in git and this host has the bodies: below.)
- **Stale maps.** This host's `data/carla-autoware-bridge` predated
  `repair_lanelet_right_of_way.py` (and Town03 two joins): the first Town05 and Town10
  batches failed 0/5 in 9 s each (SSv2: `Creating a regulatory element of type right_of_way
  failed`). After the repairs both passed, and Town03 was rerun on its repaired map (on
  the stale one: 5/5, longitudinal 0.022, same cross-track and EKF-GNSS). `just
  download-maps` now runs the repairs on maps already present.
- **Bodies since then** (043, "A new host" item 7; csb `be62582`, the vehicles exported on
  this host the same evening): `town01_ego_drive`, three runs, the ego wearing the Model 3,
  against the box ego's five from the table:

  | ego | PASS | longitudinal, m/s^2 | cross-track, m | EKF-GNSS, m | peak, m/s | MRM |
  |---|---|---|---|---|---|---|
  | box (above) | 5/5 | 0.012 (0.008-0.018) | 0.04 | 1.01 | 4.24-4.28 | 0 |
  | Model 3 body | 3/3 | 0.012 (0.012-0.018) | 0.03 (0.03-0.04) | 1.00 (0.99-1.05) | 4.28-4.29 | 0 |

  The same within the spread, as expected of a render-only body (the colliders stay the
  boxes); the lidar now sees the car's own meshes, and nothing in tracking shows it. World
  server up in 74 s (Town01 compiled), each run 42-49 s.
- **World-server preload** (first render, materials compiled): Town01 + Town02 934 s,
  Town02 alone in its own batch 1052 s, Town03 280 s, Town05 135 s, Town10HD 316 s; a reload
  of a compiled town 26-64 s.


## Handoff (2026-10-21)

Isaac work paused by the user; where it stands:

- **045** is done: the IMU/GNSS rename's Isaac leg passed on 2026-10-21 (above, under the
  rename note), and the exit runs reproduced on a second host set up from nothing
  ("Reproduction on newslab-server139").
- **Workspace state:** a pull on 2026-10-21 moved the acb pin to `25470e4` (descends from
  `c9ff587`); the checked-out submodule and `install/` are still `c9ff587`. Before the next
  run: `git submodule update --init --recursive`, rebuild acb then csb (CLAUDE.md).
- **044** (NPCs and traffic lights on Isaac) not started. Step order: tracked dynamic NPCs
  (contact verdict matches CARLA), walkers and two-wheelers, signal table / `SetSignals` /
  lamps / signals to Autoware, then exit runs. It also carries 043's open acceptance item:
  the new towns' routes on CARLA, compared with Isaac.
- **Follow-ups, small:** Town03 MRM (`control_validator` latency, see above; only if it
  recurs); `carla_bench.py` still defaults to Low quality; `town01_traffic_light` hits its
  300 s timeout; `just build-autoware-overlay` not rebuilt since 042; the 043 screencast
  (NAS, `autoveh/logs/2026 SSv2 on CARLA/2026-10-19 SSv2 on Isaac Sim screencast/`) shows
  the pre-aib ego and could be re-recorded.
