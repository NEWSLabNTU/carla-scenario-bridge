# Phase 054: Sliding on Mud and Wet Ground

Make the Isaac ego slide the way a car slides on mud, wet asphalt and soft soil, at a cost a
live simulation with several vehicles can carry. Today's model already reaches the grip limit
and slides in the right directions (052, "Today's model at the grip limit on mud"), but it
lets go sharply where a tire in soft ground builds its force slowly, the soil pushes nothing
sideways, and the car has no stability control. This phase adds those in tiers, cheapest
first, each measured for what it changes and what it costs.

**Status**: planned, 2026-10-24.

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

- [ ] **1. What PhysX lets us change while playing.** A spike setting each tire attribute on a
  running vehicle (lateral stiffness graph, longitudinal stiffness, friction-vs-slip graph,
  camber stiffness) and reading the effect back in a step steer. If the lateral stiffness
  takes, T1 can soften the tire on mud by itself; write down which do and which do not.
- [ ] **2. A limit-test suite.** `spikes/limit_mud.py` grown into the standard open-loop
  manoeuvres on asphalt, wet asphalt and mud, every frame kept: step steer (ISO 7401),
  steady-state circle (ISO 4138), double lane change (ISO 3888), split-mu braking (ISO 14512),
  braking in a turn (ISO 7975), sine with dwell (FMVSS 126). Today's numbers are the baseline
  every later step is compared with.
- [ ] **3. ESC in aib.** Beside ABS and traction control in `aib.powertrain`: a yaw-rate
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

- [ ] Which tire attributes PhysX accepts while playing, written down (step 1).
- [ ] The manoeuvre suite runs from one command on asphalt, wet asphalt and mud, and its
  results for today's model, T1 and T2 sit side by side here.
- [ ] ESC meets FMVSS 126's sine-with-dwell criteria on dry asphalt, and off it is today's car.
- [ ] On mud, the step steer's lateral force builds over a larger slip angle than on asphalt
  (the slide no longer snaps), and a car crossing ruts at a shallow angle feels a lateral force
  from them.
- [ ] No oscillation below 3 m/s on soil, at zone edges or at the hand-back (step 6).
- [ ] Four egos on soil with T2 within 10 % of 052 step 5's frame time.
- [ ] Step 8's scenarios pass on Isaac with Autoware, or their failures are explained.

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
