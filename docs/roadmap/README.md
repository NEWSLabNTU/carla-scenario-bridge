# Roadmap

Files are named `NNN-name.md`. Numbers are identifiers, not a strict execution order —
dependencies are stated in each document.

## Original plan (001-005)

Written before the multi-instance authority model. 001 and 002 are largely delivered; 003 and
004 are superseded by the audit-driven phases below.

- [001: Core ZMQ Adapter](001-core-zmq-adapter.md) — ZMQ server, CARLA connection, entity lifecycle, coordinate conversion
- [002: Autoware Integration](002-autoware-integration.md) — Sensor pipeline, ego physics readback, end-to-end driving
- [003: NPC + Pedestrian Support](003-npc-pedestrian-support.md) — superseded by [008](008-entity-fidelity.md)
- [004: Traffic Lights + Environment](004-traffic-lights-environment.md) — superseded by [009](009-map-and-traffic-lights.md); its ambient-traffic section is rejected
- [005: Hardening + AWF Contribution](005-hardening-awf-contribution.md) — upstream contribution still valid; robustness moved to [011](011-robustness.md)

## Audit-driven plan (006-011)

From the workflow audit of 2026-07-28 and the gap list in
[design/multi-instance-architecture.md](../design/multi-instance-architecture.md). Ordered by
what unblocks what.

- [006: Honest Failures](006-honest-failures.md) — stop returning success for work not done ⭐ **start here**
- [007: Repeatable Runs](007-repeatable-runs.md) — actor teardown, spawn retry, reconnection
- [008: Entity Fidelity](008-entity-fidelity.md) — pedestrians, misc objects, NPC physics
- [009: Map and Traffic Lights](009-map-and-traffic-lights.md) — map loading, signal mapping, recognition
- [010: Multi-Instance](010-multi-instance.md) — config loading, role names, background AVs
- [011: Robustness](011-robustness.md) — tick timeouts, ego respawn, duplicate-publisher hazards
- [012: SSv2-Unmanaged Autoware](012-ssv2-unmanaged-autoware.md) — SSv2 stops *launching* Autoware (`launch_autoware:=false` against our externally-launched ego stack); kills the last SSv2 patch. Spike done 2026-08-08
- [013: Unmanaged Ego on the Fork](013-forked-unmanaged-ego.md) — `managed_ego:=false` patch on the NEWSLabNTU fork; ego gets its own domain, pilot, and clock like every background AV

### Why this order

006 comes first because three handlers currently report success while doing nothing, so a
scenario suite can score fiction — no later phase can be validated until that stops.

007 came second because nothing destroyed spawned actors and spawn had no retry, so a leaked
actor blocked the next run's spawn point and the second run of any scenario failed until CARLA
was restarted. Done as of 2026-08-12; iterative work on 008-011 no longer pays that tax.

The rest follow their dependencies. 011 can be pulled forward if spurious CARLA reconnects
start appearing — 009 adds per-frame work and makes that likelier.

012's spike (2026-08-08) found that `launch_autoware:=false` is upstream's supported
"reuse a running Autoware" mode — the concealer still initializes, routes and engages, so
012 needs no pilot and no longer depends on 010. It can run as soon as a live single-ego
stack exists to verify against.

013 layers on 012 and does depend on 010's pilot: with the concealer's engage gate patched
out on the NEWSLabNTU fork, the pilot is the only thing that routes and engages the ego.
Ship 012 first — it stands alone and shrinks 013's fork patch to the minimum.

## Status (checkpoint 2026-08-12 — read this first)

**The E2E is green, and now repeatable**: `town01_ego_drive.xosc` passes on a fresh
stack and passes again on the same stack immediately afterwards (007's acceptance,
2026-08-12) — no restarts of anything between runs. The full causal
anatomy of getting there — five stacked availability bugs, the double-`/clock`
killer, PhysX acceleration jolts — is in [012](012-ssv2-unmanaged-autoware.md).
Operational recipe and traps: `docs/CHECKPOINT.md` at the repo root docs dir.

Per-phase state:

- **006 done.** 001/002 delivered. 003/004 superseded as noted above.
- **011 effectively done** — acb_bridge detects ego despawn and re-attaches to the
  next spawn without a restart (acb `aeb2031`), across map reloads and server
  replacements. Remaining checkboxes are test-harness work, not functionality.
- **007 done (2026-08-12)** — the same scenario twice in a row, both clean, nothing
  restarted between and no CARLA restart. Took two fixes found by chasing run B: the
  IMU-orphan server crash (csb destroys a vehicle's sensors first; real guard on the
  carla fork, branch `sensor-owner-guards`) and acb never noticing a despawn, because
  `Actor::IsAlive()` is client-side (acb `ab5dc67`).
- **010 COMPLETE** — two-domain run works end to end: the background AV localizes,
  routes, engages and **arrives**, while the ego passes its own scenario in the same
  run (2026-08-12, on `scenarios/town01_two_av.xosc` — the short scenario does not give
  the pilot's ~62 s cold start enough sim time). The last criterion closed 2026-08-15:
  with both AVs on lanelet 6583 the ego perceives the one ahead and **follows** it —
  closes to 18 m, slows 3.9 -> 2.5 m/s, holds ~24 m, and still passes. Teardown of the
  background AV is verified on both paths (next `Initialize`, and bridge shutdown).
- **012 DONE and verified live** — this is the architecture the passing runs use.
- **013 designed, largely unimplemented** — the deeper `managed_ego:=false` fork
  work. Run only after 007/010 close.
- **008 acceptance closed (2026-08-16)** — a pedestrian and a misc object are both
  detected by the ego's own perception, and the ego *stops* for a pedestrian crossing its
  lane and resumes when it clears (`scenarios/town01_pedestrian.xosc`). Everything comes
  back classified UNKNOWN; classification needs the camera leg 009 has parked. One test
  checkbox stays open — `resolve_blueprint_key` probes CARLA and cannot be unit-tested.
- **009 gap 7 wired, and blocked on the map pack (2026-08-16).** Traffic-light
  recognition now runs end to end — four silent wiring defects fixed, including a map
  whose lights were untyped (`subtype=""`), a camera namespace with no parameter file
  that killed the detector at startup, and acb publishing its camera on
  `/sensing/camera/camera6/camera_link/image_raw` where Autoware listens on
  `/sensing/camera/camera6/image_raw`. It still reports nothing, because every light in
  every town of the pack is a 3-14 cm stub at a fixed 135 deg bearing: sub-pixel at range
  and 45 deg outside `car_traffic_light_max_angle_range` on every approach. Not a GPU
  problem, as previously assumed.
- **009 light geometry regenerated from CARLA (2026-08-16).**
  `scripts/regenerate_light_geometry.py` rebuilds each linestring from
  `TrafficLight.get_light_boxes()` with facing taken from the referencing lanelet:
  36 of 36 on Town01, heads 0.451 x 1.221 m, facings spread across the compass. Plus a
  fifth wiring defect — acb stamped camera topics with the mounting frame rather than
  the optical one, so every projection landed behind the image plane. **Detection now
  works**: the signal is projected and found in the image continuously from 195 m out to
  7 m out. Classification is the last silent hop; the two criteria stay unticked.

**Priority order for the next sessions:**

1. **Rebuild the CARLA server from the fork** so `sensor-owner-guards` (the IMU
   null-owner guard) is actually in the running binary. csb's sensor-first teardown
   avoids the crash today, but only for actors csb destroys.
2. **A second background AV** (domain 3). Two Autoware stacks leave the ego's LiDAR at
   10 Hz on this host; a third is unmeasured, and running out shows up as a quiet crawl
   rather than an error — check `ros2 topic hz` on the ego's LiDAR, not load average.
3. **`just e2e`** — codify the bring-up + preflight gates (Startup complete,
   `change_to_stop` present, single `/clock`, no duplicate nodes) as one target.
4. **005 upstreaming batch** — play_launch lost-load rescue and compound-parameter
   fix (both on its main), SSv2 `carla-compat` arrived_goal patch, carla-fork
   exception containment and the IMU owner guard.
5. **009 / 008 / 013** in whatever order the above unblocks.

Gaps 1-3 from the design doc are fixed (deferred sync mode, `/clock` ownership,
sensor-timestamp epochs). Gaps 4-11 are distributed across 009, 010 and 011.
