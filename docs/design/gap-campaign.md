# Gap Campaign: Fuzzing for SSv2-vs-CARLA Verdict Disagreements

A scenario campaign that searches for runs where **SSv2's verdict and CARLA's physical
outcome disagree**: SSv2 passes a run in which the ego physically hit something in CARLA,
or fails a run on a collision CARLA never had. Each such run shows the backend misleading
the judge, and the campaign's job is to find them, confirm them, and reduce each to a
minimal scenario.

**Source**: user direction 2026-10-11: "go on a fuzzy scenario campaign to exploit
scenario runner to Carla gap"; scope settled in discussion the same day: the goal is
verdict disagreements (not crashes, not Autoware's own failures); the real Autoware ego is
in every run; overnight runs, several pipelines in parallel.

## Why disagreements exist

SSv2 judges a run from its own model of the world. CARLA has a different one. The bridge
is the only component that sees both: every frame SSv2 sends each NPC's intended pose
(`UpdateEntityStatus`), every spawn carries the entity's declared bounding box, and the
bridge holds the CARLA actor with its blueprint's real box and real physics. The known
sources of disagreement (catalogue: [ssv2-feature-completeness.md](ssv2-feature-completeness.md)):

| Source | SSv2 sees | CARLA has | Fixed in |
|---|---|---|---|
| Declared box vs blueprint box | the box the scenario declares (a 12 m truck) | the blueprint's box: `model3d` passes through when it names a blueprint, else `blueprint_map`, else `vehicle.tesla.model3` (4.7 m) whatever the declared size | [022](../roadmap/022-entity-geometry-parity.md): size-matched blueprint, residual reported |
| Pose reference | the entity origin, shifted by the declared `bounding_box.center` ([`OriginOffset`](../../src/carla_scenario_bridge/src/entity_manager.rs)) | the actor transform, snapped to ground; walkers lifted | [023](../roadmap/023-npc-motion-parity.md): z left to physics |
| Static world | nothing: no curbs, poles, walls | PhysX colliders for all of it | [021](../roadmap/021-physical-verdict.md): contact joined into the verdict |
| NPC motion | the NPC's speed | by default a physics body tracking SSv2's pose (p95 ≤ 0.2 m) at its real velocity; under `npc_motion: kinematic` a teleported actor at velocity 0 | [023](../roadmap/023-npc-motion-parity.md): tracked physics, default since 2026-10-13 |
| Contact | `CollisionCondition`: a 2D box overlap in its own geometry (`api.cpp:345`) | contact events between the physical ego and the NPCs' bodies (tracked by default, kinematic colliders under `npc_motion: kinematic`) | [021](../roadmap/021-physical-verdict.md) |

A disagreement is the **same frame judged differently in the two geometries**, or a contact
one of them cannot see at all.

## Contracts

What a finding violates. Two kinds.

**Differential contracts** -- SSv2 and CARLA judge the same run and must agree (CARLA as
the second implementation, NEZHA-style differential testing). Checked by the oracle
([024](../roadmap/024-gap-trace-and-oracle.md)) on every run:

| Contract | Holds when | Oracle check |
|---|---|---|
| **C1 overlap** | ego and an entity overlap in SSv2's geometry ⇔ they overlap in CARLA's | G1 |
| **C2 contact** | every CARLA contact (static world included) has an SSv2 overlap | G2 |
| **C3 pose** | intended pose ≈ actual pose (0.2 m / 2° / 0.5 m z) | G3 |
| **C4 verdict** | the JUnit verdict is consistent with C1-C3 | G4 |

**Metamorphic contracts** -- a known transformation of a scenario must change its verdict
predictably. They need no ground truth and double as root-cause probes on a confirmed
finding (campaign part 2):

| Contract | Transformation | Expected |
|---|---|---|
| **M1 placement** | move the scenario to an equivalent straight road elsewhere in the town | same verdict, same C1-C4 outcome |
| **M2 box** | set the declared box to the blueprint's box | a box-caused C1 break disappears; if not, the cause is elsewhere |
| **M3 mirror** | mirror the scenario left-right | symmetric outcome |
| **M4 prop** | replace a moving NPC by a static prop on the same poses | a static contact is still seen |

## Generation: adaptive, and exploiting what it finds

The generator borrows from greybox fuzzing in security research and from the AV fuzzers
built on it, under one constraint those tools do not have: a run costs 1-2 minutes of
Autoware, so a night is hundreds of executions, not millions. Every choice is about sample
efficiency.

| Method | Idea | Use here |
|---|---|---|
| Zest / JQF ([Padhye et al., ISSTA'19](https://arxiv.org/abs/1812.00078)) | mutate a generator's random choices, not the input; validity feedback | a scenario is a **template + parameter vector** (genome); genome mutations are structural, outputs stay valid; SSv2 rejects and spawn failures are validity feedback |
| NEZHA ([Petsios et al., S&P'17](https://www.ieee-security.org/TC/SP2017/papers/390.pdf)) | keep inputs that produce new behavioural asymmetries between implementations | the coverage signal is the run's **gap signature** (contracts broken × entity class × which geometry); a new signature enters the corpus |
| DriveFuzz ([Kim et al., CCS'22](https://arxiv.org/abs/2211.01829)), CARLA + Autoware | driving-quality feedback steers toward misbehaviour before any bug shows | gradient before the first finding: margin to contact and margin difference `d_ssv2 - d_carla` |
| AFLGo ([Böhme et al., CCS'17](https://www.comp.nus.edu.sg/~abhik/pdf/CCS17.pdf)) | distance to target, simulated-annealing power schedule | energy anneals from broad exploration toward seeds near a contract boundary |
| EcoFuzz ([Yue et al., USENIX Sec'20](https://www.usenix.org/conference/usenixsecurity20/presentation/yue)) | seed scheduling as an adversarial multi-armed bandit | which seed next: Exp3 over the corpus, reward = new signature or larger margin difference |
| MOpt ([Lyu et al., USENIX Sec'19](https://www.usenix.org/conference/usenixsecurity19/presentation/lyu)) | learn the mutation-operator distribution | operator probabilities learned per run; Thompson sampling instead of PSO at this run count |
| MAP-Elites / DeepHyperion ([Zohdinasab et al.](https://arxiv.org/abs/2107.06997)), BehAVExplor ([Cheng et al., ISSTA'23](https://arxiv.org/abs/2307.07493)) | keep the best input per feature-grid cell; behaviour diversity | archive cells: declared/blueprint size ratio × approach type × ego speed × lateral gap; keeps findings diverse |
| AV-Fuzzer ([Li et al., ISSRE'20](https://research.nvidia.com/publication/2020-10_av-fuzzer-finding-safety-violations-autonomous-driving-systems)) | GA plus local fuzzing at promising optima, restarts | small-step local search around a finding |
| ddmin / HDD (Zeller; Misherghi & Su) | 1-minimal failing input; tree variant | shrink a finding over the XOSC tree and toward round numbers |
| Digital Siblings / MultiSim ([Biagiola et al., EMSE'24](https://arxiv.org/abs/2305.08060); [Sorokin et al.](https://arxiv.org/abs/2503.08936)) | simulator disagreement marks flaky tests; use it inside the search | disagreement is our target, so flakiness must be filtered: confirm by re-run |

The loop:

```
          ┌──────────── corpus + MAP-Elites archive ◄──────────────┐
          │ (genome, signature, cell, margin diff, reward history) │
          ▼                                                        │
  seed bandit (Exp3) ─► operator bandit (Thompson) ─► mutant genome│
          │ energy: annealed, explore → exploit                    │
          ▼                                                        │
  render .xosc ─► validity screen ─► run (pipeline 1..N) ─► gap trace
                                                                   │
                                  oracle C1-C4 ─► signature, margin, finding?
                     ┌────────── no finding: reward → bandits ─────┘
                     ▼ finding
  confirm (re-run ×3) ─► bucket by signature ─► local search + boundary bisection
      ─► metamorphic probes M1-M4 (root cause) ─► ddmin/HDD shrink ─► report entry
      bucket reward decays: the search moves on to unexplored buckets and cells
```

**Adapting over time** is four loops: seed energy (bandit, decaying stale reward), the
operator mix (learned success rates), template families (bandit one level up), and the
annealing schedule. **Exploiting an error** is a fixed pipeline: confirm it reproduces;
bisect the parameter boundary where the verdict flips, which characterises the gap ("any
declared length > 6.2 m with lateral gap < 1.3 m") rather than one case; attribute the root
cause with the metamorphic probes; shrink to a minimal scenario; mark the bucket known so its
reward falls, as a crash bucket is deduplicated by stack hash. Operators: numeric nudge
(Gaussian, scaled per parameter), boundary value, entity swap (kind, `model3d`, declared
box), entity add/remove, template switch, splice of two seeds of one family.

A surrogate that screens mutants before they cost a run (ScenarioFuzz's GNN filter) is
deferred; every run's genome and outcome is logged so one can be trained later.

## Shape of the campaign

Known gaps are fixed first, each with a reproducer that disagrees before and agrees after;
then the residual is measured; then the search for unknown gaps. Each phase has its own doc
and is useful on its own.

1. **Physical verdict** ([021](../roadmap/021-physical-verdict.md)): the bridge reports
   CARLA's ego contacts per episode; the runner joins them into the JUnit as a
   `carla_contact` testcase, opt-in. Closes "SSv2 passed, the ego hit something" whatever
   the cause.
2. **Entity geometry parity** ([022](../roadmap/022-entity-geometry-parity.md)): blueprint
   chosen by subtype and declared size from a measured box table; residual reported at spawn.
3. **NPC motion parity** ([023](../roadmap/023-npc-motion-parity.md)): NPCs as tracked
   physics bodies with real velocity, within a pose budget; gated by a spike.
4. **Gap trace and oracle** ([024](../roadmap/024-gap-trace-and-oracle.md)): the
   differential contracts, checked on every run; baselined on the starter scenarios and the
   021-023 reproducers.
5. **Adaptive generator and exploit pipeline, one pipeline** (025): templates (lead brake,
   cut-in, crossing pedestrian, oncoming, parked obstacle, rear approach) as genomes; the
   bandits, archive and annealing above; validity screen; confirm, bisect, metamorphic
   probes, shrink; campaign report.
6. **Parallel isolated pipelines** (026): N complete pipelines on one host -- CARLA,
   bridge, relay, ego stack, scenario stack -- each with its own port set and ROS domains.
   Not roadmap 010, which puts several vehicles in **one** CARLA. On this shared host
   (32 cores, 60 GB, one GPU, other users' load) two is the realistic target; measured, not
   promised.

## Principles

- **SSv2 stays the judge; the physical verdict is opt-in.** Nothing the campaign adds
  changes a reply to SSv2. The one thing that can fail a run is 021's `carla_contact`
  testcase, added next to SSv2's testcases (never replacing them) and only when the runner
  is asked for it. Everything else -- trace, oracle, geometry residuals -- is diagnostic.
- **A finding reproduces.** A disagreement counts once it recurs on re-runs of the same
  scenario; one-off runs are logged, not reported.
- **Findings are inputs to fixes, not fixes.** The known gaps are fixed in 021-023 before
  the search starts; a gap the search finds later is fixed in its own phase, which the
  campaign's report prioritises.
- **Town01, CARLA 0.9.16, the proof-run stacks**, unless a phase doc says otherwise.

## Appendix: gap trace format

The per-episode record the bridge writes and the oracle reads
([024](../roadmap/024-gap-trace-and-oracle.md)): JSON Lines, one file per `Initialize`,
`<gap_trace_dir>/<UTC time>_<episode>.jsonl`, one JSON object per line, each with a `type`.
The reader is `src/csb_launch/scripts/gap_trace.py` (typed records, validation); the box
geometry the oracle judges with is `gap_geometry.py` next to it. Both are stdlib Python and
installed beside the `csb_launch` scripts.

**Conventions.** Everything is in the **ROS map frame** (x east, y north, right-handed, the
frame SSv2 works in): metres, radians, m/s. CARLA values go through the bridge's existing
conversion (Y-flip, map origin) and nothing else -- they are **raw**: no `OriginOffset`
applied or removed, so an offset the bridge got wrong is visible in the trace instead of
baked into it. A pose is `[x, y, z, roll, pitch, yaw]`. Box offsets and extents are in the
entity's or actor's own frame, x forward, y left.

**Order.** `episode` first and once; `entity` before any `frame` that names it; `frame`
numbers strictly increasing; `contact` lines anywhere after the episode line (they arrive
from `collision_monitor`'s queue, so may trail their frame); `end` last and once. Blank
lines are ignored. A trace without `end` was cut short (bridge killed, CARLA restart); the
reader rejects it unless asked not to.

**Versioning.** `schema` on the episode line, now `1`. A new optional field may be added
under the same schema; a reader rejects a schema it does not know, and every unknown
`type`, `kind` or flag, so a typo does not read as absent data.

| `type` | Field | Type | Meaning |
|---|---|---|---|
| `episode` | `schema` | int | trace schema, `1` |
| | `episode` | int | `Initialize` count of this bridge process (as the contact report's) |
| | `start_unix` | float | wall time the episode started, Unix s |
| | `npc_motion` | `tracked` \| `kinematic` | the bridge's NPC motion mode (023) |
| `entity` | `frame` | int | SSv2 frame of the spawn |
| (one per spawn) | `name` | string | SSv2 entity name; a later `entity` line with the same name is a respawn and replaces it |
| | `kind` | `ego` \| `vehicle` \| `pedestrian` \| `misc_object` | |
| | `declared.center` | [x, y, z] | declared `bounding_box.center`, entity frame |
| | `declared.dimensions` | [length, width, height] | declared box size, ≥ 0 |
| | `origin_offset` | [x, y, yaw] | the bridge's `OriginOffset`: actor origin = entity origin + R(yaw_entity)·(x, y); actor yaw = entity yaw + `yaw` (a turned prop, 022) |
| | `actor_id` | int | CARLA actor id |
| | `blueprint` | string | CARLA blueprint id |
| | `blueprint_box` | object \| null | `actor.bounding_box()` in the actor's own frame, through `prop_box_in_actor_frame` for props (022); null when that cannot be recovered (a prop at a yaw off a multiple of 90°) |
| | `blueprint_box.location` | [x, y, z] | box centre offset from the actor origin |
| | `blueprint_box.extent` | [x, y, z] | half sizes, ≥ 0 |
| `frame` | `frame` | int | SSv2 frame (`UpdateFrame`) |
| (one per SSv2 frame) | `sim_time` | float | SSv2 simulation time, s |
| | `entities` | object | entity name → state, for every live entity |
| | `.ssv2` | pose | SSv2's pose of the entity origin: an NPC's intended pose; for the ego, the pose the bridge reported back |
| | `.carla` | pose \| null | the actor's transform from this frame's world snapshot; null when the actor is missing from it |
| | `.velocity` | [vx, vy, vz] \| null | the actor's velocity, map frame; null exactly when `.carla` is |
| | `.flags` | list, optional | `spawn`: the entity's spawn frame (one snap by design); `contact_hold`: a tracked NPC held unsnapped within `CONTACT_HOLD_FRAMES` of an ego contact (023). Both make G3 divergence *info*, not a finding |
| `contact` | `frame` | int | SSv2 frame the event was seen in |
| (one per `collision_monitor` event) | `carla_frame` | int | CARLA frame of the event |
| | `sim_time` | float | s |
| | `ego_id` | int | the ego's CARLA actor id |
| | `other_id` | int \| null | the other actor; null exactly for the static world |
| | `name` | string \| null | SSv2 name of the other actor, when it is an entity |
| | `other_type` | string | the other actor's CARLA type id (`static.*` for the world), or empty |
| | `other_kind` | `ego` \| `vehicle` \| `pedestrian` \| `misc_object` \| `static` \| `unknown` | `unknown`: a CARLA actor that is no SSv2 entity |
| | `impulse` | float | normal impulse of the event, N·s, ≥ 0 |
| `end` | `frame` | int | last SSv2 frame of the episode |
| (last line) | `sim_time` | float | s |
| | `frames` | int | `frame` lines written |
| | `dropped_frames` | int | frames dropped on a full writer channel |
| | `complete` | bool | false when CARLA restarted mid-episode (contacts after it were not seen) |

Example (one line each; wrapped here):

```json
{"type": "episode", "schema": 1, "episode": 3, "start_unix": 1760000000.25, "npc_motion": "tracked"}
{"type": "entity", "frame": 0, "name": "truck", "kind": "vehicle",
 "declared": {"center": [4.0, 0.0, 1.5], "dimensions": [12.0, 2.5, 3.0]},
 "origin_offset": [4.0, 0.0, 0.0], "actor_id": 212, "blueprint": "vehicle.tesla.model3",
 "blueprint_box": {"location": [0.0, 0.0, 0.72], "extent": [2.35, 1.05, 0.72]}}
{"type": "frame", "frame": 1, "sim_time": 0.05, "entities": {"truck":
 {"ssv2": [20.2, 0.0, 0.0, 0.0, 0.0, 0.0], "carla": [24.2, 0.0, 0.03, 0.0, 0.0, 0.0],
  "velocity": [4.0, 0.0, 0.0], "flags": ["contact_hold"]}}}
{"type": "contact", "frame": 1, "carla_frame": 5021, "sim_time": 0.05, "ego_id": 211,
 "other_id": null, "name": null, "other_type": "static.pole", "other_kind": "static", "impulse": 30.0}
{"type": "end", "frame": 1, "sim_time": 0.05, "frames": 2, "dropped_frames": 0, "complete": true}
```

The oracle's two boxes per entity and frame (`gap_geometry`): SSv2's is `declared` placed at
`.ssv2` (`ssv2_box`), CARLA's is `blueprint_box` placed at `.carla` (`carla_box`); G3
compares `.carla` with `.ssv2` moved by `origin_offset` (`actor_pose_from_entity`). Touching
boxes overlap, as in SSv2's `checkCollision2D` (boost `intersects`), which also skips pairs
whose boxes do not meet in height (`z_separated`).
