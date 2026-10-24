# Phase 052: Isaac Physics, Feature-Complete

Close what the Isaac ego's physics still lacks before any of it is calibrated against
measured ground: every surface class reachable from a scenario, every body on the soil
feeling it, nothing a finished scenario leaves behind in the next one's sensors, and the
ego's own dynamics measured the way CARLA's already are. Calibration against published
measurements is [053](053-terramechanics-calibration.md); this phase changes no soil
parameter.

**Status**: in progress, 2026-10-24: steps 1-5 done, step 5 with physics egos only (several
Autoware egos on Isaac are 039's: a world server has one ego ROS domain today). Open: the
lead NPC's sinking seen in the ego's camera and lidar.

**Source**: user direction 2026-10-24: "Focus on feature completeness first. Suggest the
roadmap", then "do both phase 052 and dataset survey".

**Depends on**: [047](047-off-road-surfaces.md) (the surface and soil layers; issues
[001](../issues/001-soil-tile-rebuild-memory.md)-[003](../issues/003-world-reload-memory.md)
fixed). **Feeds**: [039](039-isaac-multi-autoware.md) (egos on soil),
[040](040-ntu-offroad-field.md), [053](053-terramechanics-calibration.md).

Not in this phase: anything 051 owns (`vehicle_check`, the spawn box check, the spec's
physics on CARLA; 051 landed 2026-10-24, moving the world server to `src/swb_isaac_world`); the NTU field's import (040); soil parameters (053); deformable ground on
CARLA.

## Where it starts

| | today |
|---|---|
| ego teardown | `IsaacWorld.despawn` deletes an ego's prims where it stands; NPCs and props are buried first (044 step 7: a prim deleted in place stays in the RTX lidar's scene) |
| ego dynamics | both simulators' step responses on flat ground measured (051 K0, K3); none on soil |
| snow | `snow_packed` (friction only) used by `town01_zones.xosc`; `snow_loose` has Wong's US snow soil row but no zone, scenario or check uses it |
| NPCs on soil | an NPC vehicle lays ruts (`SurfaceLayer.set_kinematic`, 047 S4) but stands on the zone's flat box: it does not sink and does not follow ruts (issue 001) |
| several egos on soil | one ego per run measured; the soil grid's cost per extra ego unmeasured |

## Steps

- [x] **1. Ghost egos.** Check whether a finished scenario's ego stays in the next one's
  lidar: two `town01_soil.xosc` runs on one world server, the second run's
  `/sensing/lidar/concatenated/pointcloud` searched for returns at the first ego's last
  pose. If it is there, bury the ego as `despawn` buries an NPC (kinematic, under the world,
  deleted after one step), keeping the slot path and the kept lidar (issue 003), and write it
  up as an issue.
- [x] **2. Isaac's step responses on soil.** On flat ground 051 measured them (K3,
  `src/swb_isaac_world/scripts/step_response/isaac_step.py` and `vehicle_check`): acceleration
  within 0.004 m/s^2, brakes -0.98/-2.36/-3.70 for -1/-2.5/-4, yaw rate within 1.1 % of the
  bicycle model. Here: the same steps on sand and dirt (`spikes/step_response.py`, which
  takes a surface map and a start in a zone), the soil's resistance and traction limit on
  them, beside 051's flat numbers.
- [x] **3. Snow from end to end.** A soil zone of `snow_loose` on Town01
  (`scenarios/surfaces/town01_snow.xosc` and its surface map), the ego driving through with
  Autoware; the class `unvalidated:` like mud until 053 has a reference. (Comparing its
  pad numbers with a measured tire on snow is 053's: Wong's row has no tire data.)
- [x] **4. NPCs on the soil.** A tracked or kinematic NPC vehicle on soil stands at the
  grid's sinkage under its wheels and follows existing ruts (its pose's z and pitch/roll
  from the ruts under its four wheels), without plates of its own. Checked by the world
  server's log on `town01_soil.xosc`'s lead car; not yet looked at in the ego's camera and
  lidar.
- [x] **5. Several egos on soil.** Two and four egos through one soil zone: frame time
  against one ego, ruts each ego leaves felt by the others, the world server's memory over a
  batch. Done with aib egos in one world server without Autoware (`spikes/multi_ego_soil.py`):
  039's multi-ego setup does not exist on Isaac yet, so the egos share one ROS domain and each
  gets its own command from a speed loop.

## Acceptance

- [x] No return from an earlier scenario's ego in a later scenario's lidar.
- [x] Step responses on sand and dirt beside 051's flat ones.
- [x] `town01_snow.xosc`, `town01_soil.xosc` and `town01_mud.xosc` PASS on Isaac with
  Autoware, 0 MRM, 4 runs each.
- [ ] A lead NPC on soil visibly sinks and rides in the ego's ruts.
- [x] Frame time with four egos on soil recorded; the world server's memory flat over a
  batch, as issue 003 left it (GPU flat; RSS +38 MB over 8 cycles of four egos).

## Progress

### Step 1: no ghost egos (2026-10-24)

`src/swb_isaac_world/spikes/ghost_ego.py`: Town01 in the world server's own `IsaacWorld`, a probe
lidar 15 m behind an ego's spawn, the returns inside the ego's box counted every frame through
what a scenario's end and the next one do (60 frames a phase):

| phase | most in the box | in the last frame | frames over 5 |
|---|---|---|---|
| nothing spawned | 0 | 0 | 0 |
| ego present | 141-144 | 137 | 60 |
| ego despawned (deleted where it stands, as today) | 137 | 1 | 3 |
| `LoadWorld` again | 1 | 1 | 0 |
| the next ego, at the same slot, 40 m on | 1 | 0 | 0 |

The three frames after the despawn are the lidar's latency; the ego is then gone. Burying it
first, as `despawn` buries an NPC, measured the same (136, then 1), so nothing changed. 044's
ghosts were NPCs and props, whose bodies are instanceable; the ego's is not. Why one stays
and the other does not is not known.

### Step 2: flat asphalt first, as a check of the spike against 051 (2026-10-24)

`src/swb_isaac_world/spikes/step_response.py`, aib's vehicle commanded directly (Autoware's
contract: an acceleration and a tire angle), the world server's frame of 3 x 1/60 s, sensors
off. Throttle and brake on Town01's road at y -130 from x 330; steering on the server's
ground plane (Town01's kerbs are inside a 19 m circle). Two runs identical.

| acceleration request, m/s^2 | v at 1 / 2 / 4 / 6 s, m/s | mean 1-3 s | peak | to 63 % |
|---|---|---|---|---|
| 1.0 | 0.70 / 1.70 / 3.72 / 5.71 | 1.011 | 1.055 | 0.45 s |
| 2.0 | 1.44 / 3.43 / 7.41 / 11.43 | 1.992 | 2.109 | 0.45 s |
| 3.0 | 2.14 / 5.13 / 11.13 / 17.11 | 3.003 | 3.047 | 0.45 s |
| 10.0 | 3.57 / 8.55 / 18.37 / 25.08 (at 25 m/s: stopped) | 4.951 | 5.039 | 0.45 s |

| deceleration request from 10.52 m/s, m/s^2 | stop time, s | distance, m | mean |
|---|---|---|---|
| 1.0 | 10.60 | 57.02 | 0.992 |
| 2.5 | 4.40 | 23.24 | 2.390 |
| 5.0 | 2.25 | 12.28 | 4.674 |
| 10.0 | 1.35 | 7.48 | 7.791 (ABS at the tires' limit) |

| tire angle at 5 m/s, rad (CARLA's K0 curvature, kinematically) | yaw rate, deg/s | curvature, 1/m | CARLA K0 |
|---|---|---|---|
| 0.0546 | 5.20 | 0.0182 | 5.21, 0.0182 |
| 0.1057 | 10.07 | 0.0352 | 10.08, 0.0353 |
| 0.1571 | 15.00 | 0.0525 | 14.88, 0.0527 |

Isaac realises an acceleration request to within 0.011 m/s^2 up to 3 m/s^2 (the 0.35 m/s^2
limit is Autoware's tracking through the whole loop, not this), with a 0.45 s rise (the
powertrain's lag and dead time) and a ceiling of 5.0 m/s^2 (the motor). The mean
deceleration is short of the request by 0.1 at 2.5 m/s^2 and 0.3 at 5 (the rise at the
start counts in the mean). At 5 m/s both simulators turn as a kinematic bicycle: no measurable
understeer. This agrees with 051's K3 measurement through `vehicle_check` (brakes -2.36 for -2.5 there,
-2.39 here), so the spike can be trusted for the soil runs. CARLA's K0 rows are pedal
positions, so the longitudinal columns do not compare row for row.

### Step 2: step responses on soil (2026-10-24)

`spikes/step_response.py` on 88 m of sand or dirt (`spikes/soil_steps_{sand,dirt}.yaml`,
Town01's road at y -130), each test from rest on untouched soil (`fresh_soil`), after 20
frames on the hold brake. The first runs found [issue 004](../issues/004-soil-resistance-at-rest.md):
the soil's motion resistance was applied in full at standstill, so on dirt even a 3 m/s^2
request never moved the car. With it fixed (resistance fades in below 0.1 m/s):

| acceleration request, m/s^2 | asphalt: mean 1-3 s | dirt: mean 1-3 s, v at 6 s | sand: mean 1-3 s, v at 6 s |
|---|---|---|---|
| 1.0 | 1.011 | 0.01, 0.13 m/s (creeps) | 0.00, 0.05 m/s (held) |
| 2.0 | 1.992 | 1.20, 7.2 m/s | 0.09, 0.96 m/s |
| 3.0 | 3.003 | 2.26, 13.1 m/s | 1.34, 8.4 m/s |
| 10.0 | 4.951 | 2.76, 15.9 m/s | 0.95, 6.7 m/s (wheelspin digs in) |

| deceleration request from 10.5 m/s, m/s^2 | asphalt | dirt | sand |
|---|---|---|---|
| 1.0 | 0.99 | 1.57 | 2.20 |
| 2.5 | 2.39 | 2.99 | 3.61 |
| 5.0 | 4.67 | 5.22 | 5.81 |
| 10.0 | 7.79 | 7.46 | 7.21 |

The powertrain's torque is open-loop on the request (feed-forward for asphalt), so the shortfall
is the soil's drag: about 0.75-0.8 m/s^2 on dirt (some 8 % of the car's weight) and 1.2-1.7
on sand (13-16 %), with traction ceilings of 2.8 and 1.7 m/s^2 against asphalt's 5.0 (the
motor's). A request below the drag only creeps: Autoware's controller closes the loop and asks
for more. Full throttle on sand does worse than 3 m/s^2: the spinning rear wheels dig in (slip
sinkage), as 047 found for mud. Braking gains the drag at low requests and loses a little at
the tires' limit. Whether these drags are right is 053's question (He 2020's SRTT on sand).

### Steps 3 and 4: snow, and NPCs on the soil (2026-10-24)

On Isaac with Autoware, a fresh world server each batch (newslab-server-241):

| scenario | batch | result | longitudinal, m/s^2 | cross-track, m | MRM |
|---|---|---|---|---|---|
| `town01_snow.xosc` | `play_log/acceptance/2026-10-24_14-12-02` | 4/4 PASS | 0.121-0.126 | 0.04-0.05 | 0 |
| `town01_soil.xosc` | `play_log/acceptance/2026-10-24_14-45-04` | 4/4 PASS | 0.196-0.296 | 0.03-0.05 | 0 |
| `town01_mud.xosc` | `play_log/acceptance/2026-10-24_15-02-40` | 4/4 PASS | 0.080-0.084 | 0.04-0.05 | 0 |

A first snow batch before 051's landing lost runs 2-4 to a render hang at the second ego's
warm-up (`Waiting for frame submission semaphore timed out`, device 1); the batch above, on
the rebuilt workspace, did not hang.

In `town01_soil.xosc` the lead car's body stands 7.4 cm down at its deepest in every run
("off the soil: its body was down 0.074 m at most"). The first version added the tilt op to
the NPC's instanced body on its first frame on soil: a structural edit mid-run, and the world
stood still for 2.4-3.1 s as the lead car reached and left the soil (one 3.1 s gap split a
run in two for the bag scorer, which pairs driving stretches with runs in order, so a passing
run was reported as "went nowhere"). Authored at spawn (`swb_isaac_world` `0976398`), the
gaps in the status stream fell to 1.0-1.9 s. The same batch with the sinking switched off
still had two of 1.1-1.2 s, in its first run; the batch of 2026-10-24 10:26, before 051's
landing, had none over 1 s. That remaining stall is not the sinking's and is not explained.

Also seen in every soil batch, before and after: at its start Autoware backs the ego up
about 10 m at 1 m/s (`control_cmd` velocity -1.0) away from the lead car standing just ahead,
then drives the route. It is Autoware's planner, not the vehicle; the scenarios pass.

### Step 5: several egos on soil (2026-10-24)

`spikes/multi_ego_soil.py` on `spikes/soil_steps_dirt.yaml` (88 m of dirt, two lanes at
y -129.6 and -132.6, egos heading -x from x 322), every rig sensor on, each ego held to 4 m/s
by a speed loop (it settles at 3.6 on fresh dirt, 3.8-3.9 in ruts: a proportional loop
against the drag). One world server process, newslab-server-241's RTX 5090. Frame time is the
world server's frame (3 substeps of 1/60 s, 50 ms of simulated time, a render on the last);
the soil share is `SurfaceLayer`'s update, place, begin_step and flush.

| egos | frame p50 / p95, ms | soil p50 / p95, ms | the rest, p50 |
|---|---|---|---|
| 1 | 29.2 / 35.1 | 6.2 / 7.2 | 23.0 |
| 2 (one lane) | 45.5 / 56.0 | 11.5 / 13.3 | 34.0 |
| 2 (side by side) | 45.2 / 53.0 | 11.3 / 12.8 | 33.9 |
| 4 | 77.4 / 93.3 | 21.4 / 24.1 | 56.1 |

The soil grid costs about 5.5 ms per ego per frame (0.45 ms per wheel and substep, Python),
linear in the egos; the rest (physics, four sensor sets' render and publishing) about 16 ms
per extra ego. Two egos keep real time at p50 and miss it at p95; four run at 0.65 of real
time, and would not keep it without the soil either (56 ms). The world server is lockstep,
so a slow frame slows the scenario's clock, it does not drop frames; but for four Autoware
egos at real time the soil grid's per-wheel step would have to leave Python (vectorised over
wheels, or native), and the sensor sets be reduced as 039 plans.

Ruts felt by the others, over x 255-300 (sinkage below the undisturbed surface, the four
wheels' mean; resistance the four wheels' sum; the speed loop's mean request):

| case | ego | sinkage, cm | resistance, N | request, m/s^2 |
|---|---|---|---|---|
| 1 | alone | 3.09 | 1149 | 0.62 |
| 2, one lane | first (x 310) | 3.09 | 1147 | 0.60 |
| | second, 12 m behind in its ruts | 3.86 | 437 | 0.22 |
| 2, side by side | each | 3.08-3.09 | 1148-1150 | 0.62-0.63 |
| 4, two lanes | first of each lane | 3.08-3.09 | 1146-1148 | 0.60-0.62 |
| | second of each lane | 3.82-3.88 | 323-524 | 0.18-0.26 |

An ego in another's ruts rides 0.8 cm deeper (the rut plus its own further sinkage) and meets
a third to a half of the resistance on compacted soil, as the grid's repeated loading should
give; egos side by side do not affect each other.

Memory over a batch: four egos spawned, driven 10 s and despawned, the soil made fresh,
8 times, after the cases above: RSS 17590 MB after the cases, 17628 MB after the eighth cycle
(+4.8 MB per cycle, steady), GPU 11265 MB throughout (11260 at the end). The ego slots are
reused (`_ego_path`), so the lidars cost nothing more (issue 003). The RSS creep is small (a
hundred runs: about 0.5 GB) and not traced.

### A proof run on mud, recorded (2026-10-24)

`town01_mud.xosc` on Isaac with Autoware, PASS, recorded with the world server's `record:=`
(`swb_isaac_world` `recorder.py`, off by default): the dash camera (the rig's `camera6` pose),
a still camera past the mud zone looking back along the ego's line, a camera at the rear-left
wheel, and every frame's state and per-wheel ground. `scripts/record_video.py` makes the
video: dash and trackside side by side, the wheel inset, speed, sinkage and soil resistance
below. With all four wheels on the mud they sink 2.9 cm (front) and 3.9-4.7 cm (rear) and
meet 937 N together (median; 421-1102); the speed stays at 13-14 km/h while Autoware's request
rises from 0.08 to 0.61 m/s^2 against the drag.

Two things the recording found: the soil zones rendered white (RTX ignores a mesh's
`displayColor` when it has no material), so each zone now gets a material of its class's
colour (`soil_tiles.LOOKS`); and the ruts appear in the render only once the wheels are 3 m
past a tile (`soil_tiles`' rebuild margin, which keeps the plates the wheels stand on from
changing under them), which is why the video watches them from the trackside camera and not
from the wheel. The scene's exposure is high in every camera (the sky white, the mud a pale
brown); not changed here.

### Today's model at the grip limit on mud (2026-10-24)

The proof run drove straight through the mud: the soil's drag was uneven (the right wheels
0.6 m from the zone's edge, so half of it on them), about 290 N m of yaw toward the left, and
Autoware held the lane with -0.1 to -0.22 deg of tire angle; with 0.6 m/s^2 asked and almost
no turn it used about a fifth of the mud's grip. So, without Autoware, the car was taken to
the limit: `spikes/limit_mud.py` on `spikes/limit_mud.yaml` (50 m of `mud_wet`, x 150-200,
from the road's south edge to y -129.3), one ego, its command set directly, open-loop
manoeuvres from 40 km/h after a run-up on asphalt, the same on the road's asphalt as the
baseline. Frames in `play_log/limit_mud/2026-10-24_22-15-43/`; the process segfaults in
`app.close()` after the last case is written.

| case | asphalt | mud |
|---|---|---|
| straight full braking (8 m/s^2 asked) | 8.8 m, 7.1 m/s^2, ABS idle | 16.9 m, 3.55 m/s^2, ABS 92 % of the time, straight |
| split-mu braking, left wheels in mud | | 11.4 m, 5.4 m/s^2; yaws 7.6 deg toward the asphalt side, 6.8 deg/s peak, 2.2 deg sideslip, 0.14 m sideways |
| 5 deg tire-angle step, speed held, 1.2 s | yaw rate 18.3 deg/s (0.99 of the kinematic), 3.54 m/s^2 lateral, sideslip 1.6 deg | yaw rate 14.4 deg/s mean and still rising, lateral 1.25-1.35 m/s^2 (kinematic 3.27), sideslip 6.5 deg and growing |
| 1.5 deg turn, then full braking | heading 9.4 deg at the stop, sideslip 0.5 deg | heading 22.9 deg, rear slip angle to 16 deg, yaw rate doubling to 10.8 deg/s as the car slows |

Mud's braking is 3.55 m/s^2 against the 2.3 its friction alone gives (mu 0.23 of dry): the
soil's resistance brakes too. Split-mu braking yaws the car toward the high-grip side, as it
should. The step on mud is a power oversteer: the rear wheels carry the drive against the
soil's drag (driven slip rising 0.02 to 0.15), so the rear axle's lateral share of the friction
circle shrinks, its slip angle grows from 0.3 to 6.5 deg in 1.2 s and the yaw rate does not
settle; the lateral acceleration stays near 0.13 g, about 57 % of mu g, which the rear axle's
remaining lateral grip explains (an estimate, not measured per axle). Braking in a turn on
mud spins the tail out at low speed, the brake-induced oversteer of a car without stability
control.

So today's model slides, in the right directions. What it does not have, for the next model
step: no ESC (the real Model 3 has one, so the car here spins more readily); the tire keeps
asphalt's lateral stiffness and slip curve on mud (PhysX fixes the friction-vs-slip graph once
playing; only the friction value follows the ground), so the onset is sharp where a tire in
soft soil builds its force over a larger slip angle; and the soil gives no lateral force of
its own (no lateral shear, no sidewall bulldozing, no rut walls). None of it is checked
against a measurement yet (053).
