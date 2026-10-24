# Phase 054: Sliding on Mud and Wet Ground

Make the Isaac ego slide the way a car slides on mud, wet asphalt and soft soil, at a cost a
live simulation with several vehicles can carry. Today's model already reaches the grip limit
and slides in the right directions (052, "Today's model at the grip limit on mud"), but it
lets go sharply where a tire in soft ground builds its force slowly, the soil pushes nothing
sideways, and the car has no stability control. This phase adds those in tiers, cheapest
first, each measured for what it changes and what it costs.

**Status**: in progress, 2026-10-24: steps 1-3 done (ESC in aib, off unless a rig enables it).

**Source**: user direction 2026-10-24: "I expect that the muddy road would tilt the steering",
then "I'm looking for proper physics modeling and see if we can simulate the effect in a
realistic way", then "I prefer to explore the design that balances the fidelity and
computation effort for live simulation. we may check game and physics simulation studies for
vehicle, plane, etc.", then "write up a phase doc".

**Depends on**: [047](047-off-road-surfaces.md) (the surface and soil layers),
[052](052-isaac-physics-completeness.md) (the limit test, `spikes/limit_mud.py`; the
several-ego cost spike, `spikes/multi_ego_soil.py`). **Feeds**:
[053](053-terramechanics-calibration.md) (the new soil terms' parameters are fitted there),
[039](039-isaac-multi-autoware.md), [040](040-ntu-offroad-field.md).

Not in this phase: fitting any parameter to measurements (053; values here are engineered and
say so); CARLA, whose tire model is its own; particle or MPM soil; changing the soil grid's
normal-pressure model.

## Where it starts

Measured 2026-10-24 (052), one ego without Autoware, 40 km/h, open loop:

| case | asphalt | mud (`mud_wet`, friction 0.23 of dry) |
|---|---|---|
| straight full braking | 8.8 m, 7.1 m/s^2 | 16.9 m, 3.55 m/s^2, ABS 92 % of the time, straight |
| split-mu braking, left wheels in mud | | yaws 7.6 deg toward the asphalt side |
| 5 deg tire-angle step, speed held | yaw rate 0.99 of the kinematic, 3.54 m/s^2 lateral | power oversteer: rear slip angle 0.3 to 6.5 deg in 1.2 s, yaw rate not settling, 1.25-1.35 m/s^2 lateral |
| 1.5 deg turn, then full braking | heading 9.4 deg at the stop | heading 22.9 deg, rear slip angle to 16 deg |

What the model has and lacks:

| | today |
|---|---|
| friction circle (braking and cornering share one grip) | PhysX's tire, friction written per wheel every substep from its ground |
| per-wheel ground (split-mu) | yes |
| ABS, traction control | aib's powertrain |
| stability control (ESC) | **none**; the real Model 3 has one |
| tire on mud | asphalt's lateral stiffness and friction-vs-slip curve, only the friction value scaled: PhysX refuses edits to the curve while playing. The onset of a slide is sharp |
| soil forces | resistance along the rolling direction only (`SurfaceLayer._soil`, `apply_force` at the contact); no lateral shear, no sidewall bulldozing, no force from rut walls |
| soil cost | the grid, 0.45 ms per wheel and substep in Python; 5.5 ms per ego per frame (052 step 5) |

## Design

What the survey found (2026-10-24; games, vehicle dynamics, flight and rover simulators,
PhysX's tire):

- **Games get convincing soft-surface slides from the shape of each surface's force curve,
  not from soil mechanics**: a drop from static to sliding friction (BeamNG's mud 0.55 to
  0.35), one friction budget for both directions, and either a high update rate (BeamNG at
  2 kHz) or a detailed tire run offline into tables (rFactor 2).
- **Flight simulators layer tabulated physics over a few measured anchors**: a Level D
  simulator's measured stopping-distance check is on a dry runway; wet and contaminated
  runway braking may come from tables.
- **On soft soil the lateral force has two parts** besides the tire's own: shear under the
  wheel along the slip direction and the reaction of soil bulldozed against the wheel's side,
  growing with sinkage (Ishigami et al. 2007, *J. Field Robotics* 24(3):233-250).
- **Isaac exposes no custom tire.** PhysX 5's vehicle SDK can replace the tire component, but
  `omni.physx.vehicle`'s Python binding (110.3.2 here) has no tire callback. The hooks are the
  per-wheel friction value and a force at a position on the body. Enough for a tire of our own
  on soil: PhysX's friction near zero there, our force at the contact, and the soil's reaction
  taken off the wheel's drive torque, which aib already sets every substep.
- **A lateral force applied from the last state is unstable at low speed at 1/60 s.** Our own
  estimate from the rig's numbers (cornering stiffness about 154 kN/rad per axle, about 920 kg
  on it): explicit damping is stable only above about 1.4 m/s, about 2.8 m/s with the force
  path's one-substep lag. Fixes: the lateral velocity damping semi-implicit
  (v_y / (1 + C dt / (m v))), the slip through a relaxation-length lag (time constant
  sigma / v, sigma 0.5-1.5 m, stable to far above road speeds and the soft-soil feel besides),
  and below about 3 m/s the soil's lateral force faded out and PhysX's low-speed tire in.
  Sub-cycling buys little: our hook runs once per PhysX step.

Three tiers, chosen per zone and per vehicle:

| tier | where | model | cost per wheel and substep |
|---|---|---|---|
| **T1** shaped PhysX tire | all rigid ground, every vehicle | PhysX's tire; per surface class the friction written each substep as its peak times its own curve over the wheel's combined slip (static to sliding drop, a Stribeck speed term, the water film already in `hydroplane_factor`); ESC in aib | microseconds (one value written, as today) |
| **T2** brush on soil | soil zones, the ego and physics vehicles near it | our tire: a brush of 10-20 points along the contact patch, Janosi shear (c + sigma tan phi)(1 - exp(-j/K)) along the combined slip direction with sigma from the grid's sinkage under the wheel, giving F_x, F_y and M_z; sidewall bulldozing growing with sinkage against lateral slip; a rut-wall term from the grid's height gradient across the tire; relaxation lag, semi-implicit damping, PhysX below 3 m/s | 50-100 us in Python per wheel, about 30 us batched with numpy (estimate) |
| **T3** grid-integrated shear | reference and calibration only | each grid node's shear along its own slip velocity, as Chrono SCM | 0.45 ms today, +20-40 % with direction (estimate) |

T2 is the balance point: it reuses what the grid already computes (pressure, sinkage, the
ruts, the part with history) and replaces only the shear integration, so it adds every missing
mud effect for about a tenth of the grid's cost.

## Steps

- [x] **1. What PhysX lets us change while playing.** A spike setting each tire attribute on a
  running vehicle (lateral stiffness graph, longitudinal stiffness, friction-vs-slip graph,
  camber stiffness) and reading the effect back in a step steer. If the lateral stiffness
  takes, T1 can soften the tire on mud by itself; write down which do and which do not.
- [x] **2. A limit-test suite.** `spikes/limit_mud.py` grown into the standard open-loop
  manoeuvres on asphalt, wet asphalt and mud, every frame kept: step steer (ISO 7401),
  steady-state circle (ISO 4138), double lane change (ISO 3888), split-mu braking (ISO 14512),
  braking in a turn (ISO 7975), sine with dwell (FMVSS 126). Today's numbers are the baseline
  every later step is compared with.
- [x] **3. ESC in aib.** Beside ABS and traction control in `aib.powertrain`: a yaw-rate
  target from the bicycle model capped at mu g / v, braking one wheel against the error; rig
  parameters with `esc_enabled`, so it can be compared off and on. Checked with sine with
  dwell on dry asphalt against FMVSS 126's criteria (yaw rate at most 35 % of its peak 1 s
  after the steering ends, at most 20 % at 1.75 s; at least 1.83 m lateral displacement at
  1.07 s).
- [ ] **4. T1: each surface's own curve.** Per class in `surface_classes.yaml`: peak and
  sliding friction, the Stribeck speed, a relaxation length and a cornering factor (applied
  through step 1's attribute if one takes, otherwise through the friction value). Each wheel's
  combined slip from its contact velocity (lateral slip is not in today's wheel states).
  Mud's ratio seeded from BeamNG's, wet asphalt from 047's values; `unvalidated:` as today.
- [ ] **5. T2: the brush on soil.** In `soil.py`, pure Python and numpy and unit-tested like
  the grid: the brush, bulldozing and rut-wall terms from the grid's state under the wheel.
  In `SurfaceLayer._soil`: PhysX's friction near zero by the soil's share (the zone edge's
  blend stays), our F_x, F_y and M_z at the contact; in aib a per-wheel torque the host sets,
  taken off the drive torque, for the soil's reaction on the wheel. The relaxation lag,
  semi-implicit damping and the hand-back to PhysX below 3 m/s as above. Resistance-only
  `_soil` stays available as the fallback.
- [ ] **6. Low-speed and edge checks.** Creeping across a zone edge at 0.5-3 m/s, stopping and
  holding on soil, starting from rest in a rut, one side on soil: no oscillation in yaw rate or
  lateral velocity, no jump at the hand-back speed.
- [ ] **7. Cost.** `spikes/multi_ego_soil.py` with T2 on: one, two and four egos, frame p50 and
  p95 and the soil layer's share beside 052 step 5's (29.2 / 45.5 / 77.4 ms at p50).
- [ ] **8. With Autoware.** New scenarios on Isaac, each recorded with `record:=` for the video:
  an emergency stop on mud (a pedestrian stepping out), braking with mud on only the right
  half of the lane, a curve in mud taken at the speed limit. Each passes, or fails for a
  reason written down (a slide Autoware cannot hold is a finding, not a bug).

## Acceptance

- [x] Which tire attributes PhysX accepts while playing, written down (step 1).
- [ ] The manoeuvre suite runs from one command on asphalt, wet asphalt and mud, and its
  results for today's model, T1 and T2 sit side by side here.
- [x] ESC meets FMVSS 126's sine-with-dwell criteria on dry asphalt, and off it is today's car.
- [ ] On mud, the step steer's lateral force builds over a larger slip angle than on asphalt
  (the slide no longer snaps), and a car crossing ruts at a shallow angle feels a lateral force
  from them.
- [ ] No oscillation below 3 m/s on soil, at zone edges or at the hand-back (step 6).
- [ ] Four egos on soil with T2 within 10 % of 052 step 5's frame time.
- [ ] Step 8's scenarios pass on Isaac with Autoware, or their failures are explained.

## Progress

### Step 1: what PhysX accepts while playing (2026-10-24)

`spikes/tire_runtime.py`: one ego without Autoware on Town01's asphalt at 40 km/h, each case
a fresh vehicle with one attribute of every wheel's `PhysxVehicleTireAPI` edited on the
playing stage before the manoeuvre (and once during the run-up), against the unedited car.
Frames in `play_log/tire_runtime/2026-10-24_22-39-11/`.

| attribute | edit | effect | verdict |
|---|---|---|---|
| `lateralStiffnessGraph` | value x 0.25 (68976 to 17244 N/rad) | 5 deg step: slip angles 1.6 to 4.1 (rear) and 4.7 deg (front), yaw rate at 0.3 s 15.9 to 8.2 deg/s, steady 18.3 to 16.0 deg/s, lateral 3.55 to 2.21 m/s^2; the same when edited mid run-up | **takes**, any time |
| `longitudinalStiffness` | x 0.25 (45249 to 11312) | braking at 4 m/s^2 asked: mean wheel slip 0.043 to 0.173, deceleration unchanged (3.87, 3.96) | **takes** |
| `frictionVsSlipGraph` | values halved | full braking: 7.95 m/s^2 either way; PhysX logs `Attribute "physxVehicleTire:frictionVsSlipGraph": modification not allowed once the simulation has been started. Changes will be ignored.` | **refused** (as 047 found) |
| `camberStiffness` | 0 to 1000 | none (the car has no camber), no complaint | accepted, untestable here |

So T1 can soften a tire on mud directly: the cornering and longitudinal stiffness per wheel,
written like the friction value when its ground changes. The curve's shape over slip (the drop
from peak to sliding) still has to come through the friction value, from the wheel's slip.

### Step 2: the manoeuvre suite, today's car (2026-10-24)

`spikes/manoeuvres.py` on a flat proving ground it makes at its first run (one 660 x 560 m mesh
labelled Road) with `spikes/proving_ground.yaml`'s pads: dry asphalt, wet asphalt
(`wetWithPuddles`, friction 0.75), wet mud (`mud_wet`, the soil grid with no `ground_z`, so the
wheels stay on the flat ground and there are no render tiles) and a split pad (the left wheels
in mud). One command runs it all (`PADS=`, `MANOEUVRES=` narrow it; `RIG=` overrides the rig's
vehicle fields); frames and `summary.json` under `play_log/manoeuvres/<time>/`. The driver
and inputs are fixed: the numbers compare models with each other, not with a real Model 3.

The mud pad first used the closed-form wheel, to spare the grid's memory: it meets fresh soil
under every wheel at every step, and held the car to 26 km/h at 3 m/s^2 asked (braking read
6.5 m/s^2 against the grid's 3.5). On the grid the pad agrees with Town01's mud: straight
braking 16.1 m and 3.52 m/s^2 (Town01 16.9 m, 3.55), split-mu braking yaws 7.65 deg (7.6).

### Step 3: ESC (2026-10-24)

`aib.esc` (aib `1d49569`): the bicycle model's yaw rate at the present speed and tire angle,
capped at 0.85 mu g / v, against the measured one, with a dead band of 0.035 rad/s plus 10 % of
the target; a sideslip past 0.07 rad counts as oversteer whatever the yaw rate. Oversteer
brakes the outer front wheel and cuts the motor; understeer brakes the inner rear. The yaw
moment (12000 N m per rad/s of error, 30000 per rad of sideslip) becomes one wheel's brake
torque (at most 2000 N m) through the hydraulics' lag, added ahead of ABS, so ABS still keeps
the wheel turning. The friction is the one the host writes under the wheels: an idealised ESC,
where a real one estimates it. A rig's `esc_*` fields, from a spec's `brakes.esc`; off by
default, and the shipped Model 3 spec has none yet. Unit tests in `test/test_esc.py`.

The suite without and with it (`RIG='{"esc_enabled": true}'`; `play_log/manoeuvres/
2026-10-24_22-44-02` dry and wet, `2026-10-24_22-50-32` mud and split, `2026-10-24_22-55-47-esc`):

| case | without ESC | with ESC |
|---|---|---|
| dry sine with dwell, 80 km/h (6.67 deg) | yaw rate 43 % of its peak 1 s after the steering ends, 0.7 % at 1.75 s: **fails** FMVSS 126 (35 %, 20 %); heading 51 deg, sideslip 13 deg; 2.77 m lateral at 1.07 s | 0 % and 0 %: **passes**; heading 15 deg, sideslip 6.2 deg; 2.69 m (at least 1.83) |
| wet sine with dwell, 80 km/h | 92 % and 88 %: a spin, heading 82 deg, sideslip 34 deg | 0 % and 0 %; heading 12.6 deg, sideslip 5.3 deg; 2.32 m |
| dry / wet double lane change, 80 km/h | path error 0.77 / 0.86 m | 0.66 / 0.71 m, 78 km/h at the end |
| mud step steer, 40 km/h | sideslip 10.8 deg and growing, yaw rate 1.15 of the kinematic | sideslip 2.6 deg, yaw rate 0.67 of the kinematic (held to what the mud carries) |
| mud double lane change, 40 km/h | spins: sideslip 42 deg, path error 3.5 m | path error 0.19 m, 31 km/h at the end |
| dry / wet / mud circle, 40 m | the most lateral acceleration held 8.41 / 7.35 / 1.7 m/s^2 | 7.26 / 6.49 / 1.51: ESC steps in at 0.85 mu g |
| straight braking, steps on dry and wet | | unchanged (to the centimetre) |
| mud braking in a turn | heading 18.3 deg, sideslip 5.4 deg | 18.8, 5.7: unchanged |
| split-mu braking | yaws 7.65 deg | 7.66: unchanged |

Two things it does not do. On split-mu braking it asks for a moment against the yaw by braking
the front wheel on the mud side, where ABS already holds the brake at the mud's limit, so
nothing changes; a real ESC (with ABS's yaw-moment limitation) lowers the brake on the
high-grip side instead. And braking in a turn on mud is unchanged for the same reason: every
wheel is at ABS's limit. Both are for a later step (taking brake off rather than adding it);
neither matters with ESC off, which stays the default until it is decided for the shipped car.

The circle also shows the car steering neutrally: an understeer gradient of 0.06 deg/g on dry
asphalt and -0.04 on wet (the yaw rate 0.99 of the kinematic in the step). A road car
understeers; this is the rig's tires (equal stiffness per load front and rear), for T1.

## Open questions

- Whether the soil's reaction on the wheel can stay a drive-torque offset, or the wheel's spin
  needs our own integration on soil (PhysX's wheel with near-zero friction and our torque).
- Where a vehicle hands T2 back to T1 when several are on soil and the frame budget is short
  (distance from the ego, or the ego only).
- Mud has no measured lateral data that the survey found; El-Razaz 1988, Ishigami 2007 and
  Virginia Tech's soft-soil tire tests are on sand and loam, so mud's T2 values stay engineered
  after 053 too.

## Sources

- Ishigami, Miwa, Nagatani, Yoshida, "Terramechanics-based model for steering maneuver of
  planetary exploration rovers on loose soil", *J. Field Robotics* 24(3):233-250, 2007.
- Senatore, "Prediction of mobility, handling, and tractive efficiency of wheeled off-road
  vehicles", Virginia Tech thesis, 2010 (https://vtechworks.lib.vt.edu/handle/10919/37781).
- ISTVS, "Systematic tests for study of tire tractive performance on soft soil, part I"
  (https://www.istvs.org/publication-news/systematic-tests-for-study-of-tire-tractive-performance-on-soft-soil-part-i-experimental-data-collection).
- Wong, *Theory of Ground Vehicles*; Pacejka, *Tire and Vehicle Dynamics* (brush model,
  relaxation length).
- ISO 7401, 4138, 3888, 14512, 7975; FMVSS 126 (ESC, sine with dwell).
- NVIDIA PhysX 5 vehicle SDK documentation (tire component); `omni.physx.vehicle` 110.3.2.
