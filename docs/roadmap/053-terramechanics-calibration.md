# Phase 053: Soil Calibration Against Published Measurements

Calibrate the soil classes against measured ground rather than against other simulators.
047 S5 fitted sand and dirt to Chrono SCM, and Chrono CRM disagreed with SCM on traction;
only measurements decide between them. Mud (Wong's clay row, `unvalidated:`) and snow have
not been checked against anything. This phase takes the measurements from published
datasets, so it does not wait for the NTU field (040), whose cone index and drawbar test
become the local check at the end.

**Status**: planned, 2026-10-24. Step 1 (the survey) under way.

**Source**: user direction 2026-10-24: "Does soil calibration require real data? If yes, I
prefer to find real dataset online. Same for mud and snow cases. We can do dataset survey and
keep them noted."

**Depends on**: [047](047-off-road-surfaces.md) S5's rig (`spikes/crm_calibrate.py`),
[052](052-isaac-physics-completeness.md) (snow reachable from a scenario). **Feeds**:
[040](040-ntu-offroad-field.md), [041](041-competition-director.md).

## Steps

- [ ] **1. Survey.** [design/terramechanics-datasets.md](../design/terramechanics-datasets.md):
  for each class (dry sand, sandy loam, wet clay as mud, snow), the published measurements
  (bevameter, shear, cone index, single-wheel and vehicle tests of pneumatic tires: pull,
  resistance and sinkage against slip), with soil state, tire, load, format and access.
- [ ] **2. One reference per class.** A primary and a fallback dataset per class, preferring
  a pneumatic tire near the Model 3's 4.5 kN wheel load; obtain the data (digitise a plot
  when only a plot is published, and say so).
- [ ] **3. Fit.** The single-wheel rig on our grid at the reference's tire and load, against
  the measured curves; fit what 047 S5 fitted (Janosi K, slip sinkage) and, where the
  reference measures it, the pressure-sinkage row. A table per class: measured, ours before,
  ours after.
- [ ] **4. Drop the flags.** A class within tolerance (pull within ±20 % over slip 0.1-0.5,
  sinkage within ±25 %) loses `unvalidated:`; one that is not keeps it, with the reason.
- [ ] **5. Re-run.** `town01_soil.xosc`, `town01_mud.xosc` and `town01_snow.xosc` on Isaac
  with Autoware after the fit, against 052's runs.
- [ ] **6. The field (040).** Cone index and a drawbar test of a road tire on the field's
  sand and dirt, held to the same tolerance, when the field is available.

## Acceptance

- [ ] Every soil class names its reference in `surface_classes.yaml`.
- [ ] The per-class table in this doc; each class calibrated or flagged with the reason.
- [ ] The three soil scenarios PASS after the fit.

## Progress
