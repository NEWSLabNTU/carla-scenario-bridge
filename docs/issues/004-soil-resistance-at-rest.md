# 004: Soil resistance holds a car at rest that its drive should move

**Severity**: medium. **Status**: fixed (2026-10-24): the resistance fades in below 0.1 m/s
(`surface_layer.resistance_scale`). **Found**: 2026-10-24, roadmap 052 step 2 (the step
responses on soil).

## What is wrong

`SurfaceLayer._soil` pushes each wheel's soil motion resistance (compaction and bulldozing,
`WheelSoil.resistance`) onto the body as an external force at the contact, every substep, at
its full size, and takes its sign from the body's longitudinal speed:

```python
f = -out.resistance * share * (1.0 if speed >= 0 else -1.0)
```

Rolling resistance acts against motion that is happening. At rest it can hold a car, never
push it: it is bounded by what drives the car. Applied in full at zero speed, it pushes the
car backwards whenever the drive is smaller, the speed turns negative, the sign flips, and the
car dithers about zero. A drive larger than the resistance moves it, but the full resistance
is still subtracted from the first instant, as if the car were already rolling.

## How it was seen

`spikes/step_response.py` on 88 m of dirt (sandy loam, `spikes/soil_steps_dirt.yaml`), aib's
Model 3 commanded directly from rest, after 20 frames standing on its hold brake:

| acceleration request, m/s^2 | speed at 1 / 2 / 4 / 6 s, m/s | peak acceleration |
|---|---|---|
| 1.0 | -0.00 / -0.01 / -0.01 / -0.01 | 0.28 |
| 2.0 | 0.01 / -0.01 / 0.02 / 0.01 | 0.54 |
| 3.0 | -0.01 / -0.00 / -0.01 / 0.02 | 1.23 |
| 10.0 | 0.24 / 0.63 / 5.37 / 11.13 | 2.99 |

The same car pulled 3 m/s^2 on asphalt to within 0.003. Once rolling (the brake test gets to
10.5 m/s on the same dirt) it brakes as expected, the soil adding to the brakes (1.56 m/s^2
for a 1.0 request). The speed dithering about zero is the sign flip. The powertrain's torque
is open-loop on the request (`aib.powertrain.feedforward`), so a request of 3 m/s^2 gives the
drive force of 3 m/s^2 on asphalt, and the full resistance of the static sinkage takes it.

## Why it matters

- A car stopped on soil (a stop line in a zone, a queue, a scenario that starts on soil)
  cannot drive off unless something asks for much more than it needs. Autoware's controller
  integrates its error, so it eventually asks for more; a scenario still sees a car stuck for
  seconds.
- 047's mud result rests on it: "The rear-driven Model 3 cannot pull away from rest in mud"
  (design doc, pad table: "0.02 m/s: stuck, dug in to 5.7 cm, 1941 N"). Part of that is slip
  sinkage, which is real, and part may be this.

## Fix

Fade the applied force in with speed, as a regularised Coulomb friction does: the full
resistance from a small speed `RESIST_V0` up, proportionally less below it, none at rest.
`RESIST_V0` is 0.1 m/s. A drive below the resistance then gives a creep below 0.1 m/s,
which is this model's error in place of a car pushed backwards. The grid's sinkage, thrust and
traction limit are unchanged. Then re-run the step responses on sand and dirt and 047's mud
pad case, and correct the design doc's mud claim to what the fixed model says.

## After the fix

The same steps (each from untouched soil; the first runs above had each test start in the
last one's ruts, which made sand look worse than it is): on dirt a 2 m/s^2 request now gives
1.20 and 3 gives 2.26, the drag about 0.8 m/s^2; a 1 m/s^2 request creeps at 0.13 m/s,
below the drag. Tables in [052](../roadmap/052-isaac-physics-completeness.md), step 2.

047's mud pad (`spikes/soil_drive.py`, `SOIL_SPIKE_ONLY=mud`) again: coasting, holding speed
and braking are as before (0.67 m/s^2; 9.53 then 9.87 m/s; 30.75 m). Full throttle from rest
is still stuck, 0.07 m/s after 3 s (0.02 before), dug in to 5.5 cm against 1941 N of
resistance before and 2875 N now: the rear tires' soil-limited friction (0.198) on the rear
axle's load gives about 1.8 kN, less than the resistance, so it is traction, not this bug,
that holds it. The design doc's claim stands, with these numbers.
