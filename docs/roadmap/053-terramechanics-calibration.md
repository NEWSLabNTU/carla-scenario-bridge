# Phase 053: Soil Calibration Against Published Measurements

Calibrate the soil classes against measured ground rather than against other simulators.
047 S5 fitted sand and dirt to Chrono SCM, and Chrono CRM disagreed with SCM on traction;
only measurements decide between them. Mud (Wong's clay row, `unvalidated:`) and snow have
not been checked against anything. This phase takes the measurements from published
datasets, so it does not wait for the NTU field (040), whose cone index and drawbar test
become the local check at the end.

**Status**: in progress, 2026-10-24. Step 1 (the survey) done.

**Source**: user direction 2026-10-24: "Does soil calibration require real data? If yes, I
prefer to find real dataset online. Same for mud and snow cases. We can do dataset survey and
keep them noted."

**Depends on**: [047](047-off-road-surfaces.md) S5's rig (`spikes/crm_calibrate.py`),
[052](052-isaac-physics-completeness.md) (snow reachable from a scenario). **Feeds**:
[040](040-ntu-offroad-field.md), [041](041-competition-director.md).

## Steps

- [x] **1. Survey.** [design/terramechanics-datasets.md](../design/terramechanics-datasets.md):
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

### Step 1: the survey (2026-10-24)

[design/terramechanics-datasets.md](../design/terramechanics-datasets.md). Almost no open data
measures a passenger tire on soft soil. The exception, and the primary reference for sand, is
Virginia Tech's single-wheel rig with the Standard Reference Test Tire (P225/60R16) at 5 kN on
dry loamy sand (He 2020, open on VTechWorks): a peak drawbar-pull coefficient of 0.12 at about
12 % slip at 290 kPa, between treaded SCM's (about 0.3) and CRM's (below zero), which is the
disagreement 047 S5 left open. The picks for step 2, as the survey makes them:

| class | primary | access | what it lacks |
|---|---|---|---|
| sand | He 2020, SRTT on dry loamy sand | open | walking pace (6 cm/s), 1 % moisture |
| dirt | NG-NRMM 2018 Keweenaw bevameter, cone index, moisture | public (Distribution A) | a 12 t truck's tire: soil parameters only |
| mud | WES TR 3-666 (Wismer 1966), a 9.00-14 smooth tire on near-saturated fat clay | public domain | a tractor-size tire; stays `unvalidated:` |
| snow (fresh) | Lee et al. 2012, J. Terramechanics 49(5) | paywalled | the tire unverified |
| snow (packed), ice | CRREL 93-9 (Shoop 1993); Virginia Tech's ice data (Zenodo 1009017, CC BY 4.0) | public domain, open | |

`snow_loose` is exactly Wong's US snow row in SI units; its Janosi K is not from the table.
The survey's own unverified items (Wong's values from the literature, not the book; DTIC and
ERDC reports from catalogue records; some papers' tires) are marked there.
