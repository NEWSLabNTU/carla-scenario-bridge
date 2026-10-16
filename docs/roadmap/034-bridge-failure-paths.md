# Phase 034: Bridge Failure Paths

What the bridge does when CARLA is slow or stuck: a tick that never returns, a spawn that
times out after CARLA has created the actor. Today the first can wedge a run silently and
the second leaves a stray `hero` on the road; both must end in a clear error or a clean
recovery, inside SSv2's request timeouts, with no stray actor left behind.

**Source**: 032 step 7 (the 12-min hung tick), 032 step 8 N5 (the tick watchdog), 033 step
4b (a 31 s ego spawn that leaked actor 9004 and stacked the retry on it); two read-only
studies 2026-10-15 -- csb and the SSv2 fork, then the CARLA 0.9.16 source (tag `0.9.16`,
`~/repos/carla-0.9.16`) and carla-rust 0.15.0. User direction 2026-10-15: study before
fixing; do steps 1-5, then step 6 (the LibCarla rebuild in carla-rust).
**Blocks**: [033](033-demo-tour.md) step 5 (the tour spawns and despawns more than any run
so far). **Depends on**: [032](032-npc-tracking-fixes.md) step 8 (watchdog, exit 75).

## What the CARLA source says

Paths below are in the 0.9.16 tree (`TML` = `LibCarla/source/carla/trafficmanager/
TrafficManagerLocal.cpp`, `Sim` = `LibCarla/source/carla/client/detail/Simulator.cpp`,
`CS` = `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp`).

1. **The hung tick is a lost wakeup in the Traffic Manager (TM), in csb's own process.**
   In synchronous mode every `World::Tick` (`Sim:227-242`), after the frame arrives, takes
   the static `TrafficManager::_mutex` and runs `SynchronousTick` on each TM. That sets
   `step_begin` and notifies **without holding `step_execution_mutex`** (`TML:253-254`),
   then waits on `step_end` with no timeout (`TML:256-257`); the worker's `Run` does the
   mirror image (`TML:157-159`, `TML:241-242`). A notify that lands between the other
   side's predicate check and its wait is lost and both threads wait forever. The same
   pattern is in `Stop()` (`TML:265-272`) and `SetSynchronousMode` (`TML:471-473`).
   `set_synchronous_mode_time_out` is never read (`Parameters.cpp:16,152`).
   Upstream removed the handshake after 0.9.16: commit
   [`e7e076f`](https://github.com/carla-simulator/carla/commit/e7e076f1b7c98fd13364ecd73663c5bbbb0d1cf8)
   ("Removed TM mutex", 2025-10-15, `ue4-dev`) runs `Step()` inline in sync mode.
2. **No TM, no hang.** With no TM registered `TrafficManager::Tick` loops over nothing.
   csb creates the TM (`instance_tm(8000)`, `coordinator.rs`) for the first
   `simulator_autopilot` entity and keeps it until a reconnect, so every later scenario in
   that process is exposed.
3. **`apply_settings` reaches the handshake too**: `World::ApplySettings` runs up to 30
   `World::Tick`s when `fixed_delta_seconds` is set (`World.cpp:56-84`, `Sim:282-285`).
   csb's `enable_sync_mode`, `restore_async_mode` and shutdown call it, unwatched.
4. **In-process recovery is impossible.** The stuck thread holds the static TM mutex
   (`TrafficManager.cpp:63`), which every later tick, settings change and TM call takes.
   Killing csb frees it; nothing in CARLA's server needs freeing. A new process's
   `instance_tm(8000)` finds the dead TM still registered, gets "connection refused" and
   starts a local one (`TrafficManager.cpp:176-233`).
5. **CARLA never leaves synchronous mode on its own** -- no disconnect hook, no notion of a
   sync client (`CarlaEngine.cpp:384-388`). A fresh client in sync mode with nobody ticking
   sees an empty `get_actors()` (`Sim:141-143`, `Episode.cpp:129-131`). The respawned
   bridge sees actors only because its `Initialize` enables sync mode first, and
   `ApplySettings` ticks.
6. **A timed-out spawn completes on the server.** RPCs are queued to the game thread and run
   whether or not the client still waits (`CS:725-746`); in asynchronous mode the game
   thread serves RPCs for 1 ms per frame (`CarlaEngine.cpp:297-301`) -- the 31 s ego spawn
   ran in async mode on a loaded, swapped host. `World::TrySpawnActor` swallows every
   exception (`World.cpp:136-147`) so a timeout reads as a collision, and carla-rust only
   exposes that path.
7. **Actor ids only increase** (static `ID_COUNTER`, never reset, `ActorRegistry.cpp:24,
   85-94`); `role_name` and `type_id` are readable by id at once (`get_actors_by_id`,
   `CS:708-723`), the pose only after a tick. No RPC lists all actor ids. **But LibCarla
   answers `World::GetActors(ids)` from the client's `CachedActorList` first and asks the
   server only for ids it has never seen** (`Episode.cpp:29-35`), and that cache is never
   evicted (`CachedActorList.h`; its only `Clear` is in the uncalled `OnEpisodeStarted`,
   `Episode.cpp:133-134`). So csb's own long-lived client still "finds" an actor destroyed
   since it saw it: by-id lookups that must be authoritative go through a short-lived
   client of their own (step 5; added 2026-10-15 while implementing, from the source).
8. **`Actor::Destroy` asks the client's episode state first** (`Actor.cpp:149-160`): an
   actor the client has not seen in a frame -- created since its last tick, or found by a
   client that never ticks -- reads as `Invalid` and is refused ("attempting to destroy an
   actor that is already dead") without an RPC. A batch `DestroyActor` is decided by the
   server. (Added 2026-10-15 from step 5's first live run.)
9. **`apply_batch_sync` separates the cases**: a collision is a per-command error ("Spawn
   failed because of collision at spawn position", `ActorSpawnResult.cpp:22-23`), success
   carries the id, a timeout fails the whole call (`Client.cpp:753-758`). carla-rust has it
   (`Client::apply_batch_sync`, `Command::spawn_actor`).

The earlier csb-only study had two things wrong, corrected above: CARLA does not revert to
async when the bridge dies, and `apply_settings` is not a single silent frame wait.

## Intended behaviour

| Event | csb | SSv2 | ego (acb, relay) | CARLA |
|---|---|---|---|---|
| A tick or settings change does not return within the watchdog | reply with an error naming the step and "bridge restarting"; **drain**: answer every later request at once with the same error, touch nothing in CARLA; `_exit(75)` when idle ~2 s (at most ~10 s), socket flushed first | records the error verdict; its deactivate `DespawnEntity` gets the drain error, not a 90 s timeout | keeps running; `/clock` pauses | unchanged, still synchronous |
| Respawned csb, next `Initialize` | enables sync (ticks), reaps leftovers found **by id**, no TM unless the scenario needs one | normal run | acb rebinds to the new `hero` | -- |
| A spawn times out | look up ids above the pre-spawn watermark, match type and role, **adopt** the actor; never climb the lift ladder after a timeout; whole spawn bounded ~75 s | normal run | binds the one `hero` | the late spawn exists once |
| A spawn is blocked | lift ladder as today | -- | -- | -- |

## Timeouts after this phase

| Bound | Value | Why |
|---|---|---|
| CARLA client RPC | 30 s (120 s around `load_world`) | unchanged |
| LibCarla frame wait in `Tick` | **10 s** (`LIBCARLA_TICK_TIMEOUT`) | 5x the worst real tick (1.90 s) |
| Tick watchdog | **45 s** | above RPC 30 + frame 10, so LibCarla's own errors stay ordinary errors |
| State read after expiry | 10 s | 45 + 10 = 55 s < SSv2's 90 s `SIMULATOR_RESPONSE_TIMEOUT` |
| One spawn request, all attempts | **~75 s** | < 90 s |
| `apply_settings` watchdog | **45 s** | `set_episode_settings` 30 + 1 s frame sync + its ticks (frame wait 10 each) |
| `load_world` watchdog | **150 s** | above the load's 120 s RPC timeout |
| TM `ShutDown` / `SetSynchronousMode` | **45 s** watchdog | same handshake as the tick |
| `Initialize` | deadline **280 s** (+10 s state read < 300 s `SIMULATOR_INITIALIZE_TIMEOUT`) | settings and map load watched, each capped by what is left; a step past it fails `Initialize` naming it |

## Steps

1. [x] **No TM unless needed.** At `Initialize` (and when the last autopilot entity is
   despawned) shut the TM down if the coming scenario has no `simulator_autopilot`;
   autopilot vehicles are destroyed or parked first (they keep their last control). The
   TM is recreated lazily (~4 s) and light states re-applied as today. (`Initialize`
   carries no entity list, so csb cannot know what the coming scenario spawns: it shuts
   the TM down at every `Initialize` -- after the previous run's teardown none is left --
   and lets the first `simulator_autopilot` spawn start one. The shutdown runs under the
   watchdog: `Stop()` joins the worker through the same handshake.) Test: unit tests
   on the decision; live, `traffic_light` then `simulator_autopilot` then `traffic_light`
   in one bridge process, the log shows TM up only for the second.
2. [x] **Drain and exit after a hang.** After a hung tick or settings change, csb answers
   every request with the stored error without touching CARLA (no `restore_async_mode`,
   no `apply_settings`, no TM call), closes the ZMQ socket with a linger so replies flush,
   flushes its logs and `_exit(75)`s once idle ~2 s or at ~10 s. A **test-only fault
   hook** (env `CSB_TEST_HANG_TICK_AT=<frame>`, off by default) blocks the tick job
   forever. Live test T1: one scenario hung at a frame, then the next through `run_suite`:
   the error verdict in junit within ~55 s, SSv2's deactivate despawn answered (no 90 s
   wait), exit 75, respawn, next scenario passes, acb on the new `hero`.
3. [x] **Watch every call that can tick, and fit the budgets.** `apply_settings` (sync on,
   async restore) and `load_world` under the watchdog with their own named bounds; frame
   wait 10 s, watchdog 45 s (replaces the uncommitted 40 / 20 s change); `Initialize`'s
   steps sum under 300 s; the 032 doc's "twice the frame wait" reasoning and the
   design-doc timeout table corrected. Test: unit tests on the bounds; the T1 run again.
4. [x] **Spawn: tell a timeout from a collision, adopt the late actor.** Spawn through
   `apply_batch_sync` (`Command::spawn_actor`); a collision error climbs the lift ladder
   as today; a timeout scans `actors_by_ids` above the id watermark taken before the
   spawn, matches `type_id` plus `role_name` (`hero` for the ego, `csb_entity:<name>`
   for NPCs), and adopts the match; no match is an error, never a lift. The whole request
   is bounded ~75 s. (Only CARLA's collision error lifts; its other per-command errors --
   invalid description, unknown -- fail at once, since a lift cannot help them. The lookup
   is an RPC queued behind the spawn on the game thread, so it sees the spawn if the server
   ran it. The 75 s is a request deadline: each attempt's RPC gets min(30 s, what is left
   less 10 s for the lookup and 20 s for the settling tick), the settling tick's watchdog
   is capped by what is left (floor 10 s), and the RPCs after the spawn by the same.) Test: unit tests with a fake world (timeout then late actor; collision
   then lift); live, a spawn with a forced short client timeout on a busy server adopts
   the actor and leaves exactly one `hero`.
5. [x] **Look actors up by id.** The `Initialize` reaper and the post-spawn occupancy check
   (`vehicle_occupying`) enumerate `actors_by_ids` over `[1, watermark + margin]` instead
   of the cached snapshot list (through a short-lived client: csb's own answers from its
   never-evicted actor cache, finding 7; and what it finds is destroyed by a server-side batch
`DestroyActor`, children first, since `Actor::destroy` refuses actors that client never saw
in a frame, finding 8), so they see actors created or destroyed since the last
   tick (no phantom "Destroyed orphaned" lines). Test: unit tests; live, the reaper's log
   after T1 lists exactly the dead process's actors.
6. [x] **Fix the cause in LibCarla: carla-rust's prebuilt `libcarla_client`.** Backport
   upstream `e7e076f` (inline `Step()` in synchronous mode, no handshake) onto 0.9.16,
   rebuild the prebuilt client library carla-sys downloads (`carla-sys build.rs` prebuilt
   path; `build-prebuilt` with `CARLA_DIR` on a patched tree), publish it from the
   carla-rust project (pull and push to its `main`, linear history), bump csb and acb to
   the release together. Then a sync TM stress run (TM in sync mode, tight tick loop,
   CPU contention) on a second CARLA instance shows no hang where 0.9.16 stock hangs or
   the rate is measured. Steps 1-5 stay: they bound whatever else can stall.

## Acceptance

- T1 passes: a forced hang ends in an error verdict within ~55 s, the bridge restarts, the
  next scenario passes, no stray `hero`, no 90 s SSv2 timeout in the logs.
- A scenario without `simulator_autopilot` runs with no TM in the bridge process.
- A timed-out spawn leaves exactly one actor for the entity, adopted.
- `just test` green; the Town01 starters 6/6 and 033's five incident templates still pass.
- After step 6: the stress run shows no TM hang.

## Results

**2026-10-15 -- steps 1-5 done; acceptance (but step 6) met: T1 passes, no TM without
`simulator_autopilot`, a timed-out spawn adopted with exactly one `hero`, Town01 starters 6/6,
incidents 5/5.** Code: `budgets.rs` (new: every bound, `Deadline`, tests on the arithmetic),
`tick_watchdog.rs` (one untyped helper for every blocking call; the test hang hook),
`spawn_ladder.rs` (new), `actor_census.rs` (new), `zmq_server.rs` (drain, flushed close),
`main.rs` (`libc::_exit(75)`), `autopilot.rs` (`traffic_manager_action`), `coordinator.rs`.
Tests written first and seen red (a missing item, or a stub of today's behaviour) before the
code: 3 TM decision, 4 drain/hook, 6 budgets (2 red at the old 40/20 s), 7 ladder (6 red
against the old "every failure lifts" loop), 6 census, 2 hook/collision classification.
`just test` 328 nextest + 174 pytest, `just check` clean. Live: CARLA 0.9.16 Epic, Town01,
`run_suite`, three ego stacks (`just ego-av`, each < 60 min), host load 25-100 (another user's
`FVP_BaseR_AEMv8` and builds; 234 at one peak during the build). Scratch:
`/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-80a742b40cbd/scratchpad/s034`
(`runs/`, `bridge_log_sim*/`, `sim5_bridge_tail.out`, `windows.txt`).

*Step 1.* `retire_traffic_manager` at `Initialize` (after teardown) and after the despawn of the
last `simulator_autopilot` entity; every TM `ShutDown`/`SetSynchronousMode` runs under the
watchdog. Live (load 54-98): `traffic_light`, `simulator_autopilot`, `traffic_light` in one
bridge process, 3/3 PASS; the log has `Traffic Manager on port 8000: synchronous=true` only at
the second's `npc_autopilot` spawn and `Traffic Manager on port 8000 shut down (despawn of
'npc_autopilot': no simulator_autopilot entity left)` at its despawn; no TM line in the first
or third. The acceptance run shows the same within the starters.

*Step 2.* After a hung call `ZmqServer` drains (`DrainPolicy` idle 2 s, max 10 s): every request
gets `csb is restarting after a CARLA call that never returned; this request was not run:
<reason>` in its own variant, CARLA untouched; then the socket closes with a 1 s linger, the
context's termination waits it out, the gap trace closes, stdout/stderr flush and
`libc::_exit(75)` (no static destructors, which could wait on the TM mutex). Test hook
`CSB_TEST_HANG_TICK_AT` (+ `_TOKEN`), off unless set, warned at start-up. Live T1 (pedestrian
hung at frame 40, then `engage_state`, then `ego_drive`): frame 40's tick 07:55:11, the error
at 07:55:56.17 (`waited 45.0 s, bound 45 s`, CARLA frame and settings, `Traffic Manager not
running`), SSv2's deactivate `DespawnEntity` answered 2 ms later, `Drained 1 request(s)`,
exit 75 at 07:55:58.18 (play_launch: `Exited with code 75`), respawned bridge bound 5555 at
07:56:00; the junit holds `SimulationError ... failed frame` after 62 s; no 90 s SSv2 timeout;
the next two PASS (48 s, 42 s); acb dropped the reaped `hero` and bound the new one
(`Found vehicle: ID=9443`). A first T1 run (ego_drive hung, then engage_state; the 45 s
build) gave the same timeline and PASS, but play_launch truncated the node's `out` on
respawn, so its dead-process log was lost; the later runs tailed it. **Fixed in play_launch
`90c253bc`** (main, after 0.18.2; play_launch issue #0075): a respawn keeps the previous
process's `out`/`err`/`pid`/`status` as `out.N`/`err.N`/`pid.N`/`status.N` (start order),
the plain names stay the live process's. Verified 2026-10-16 with that build on the
simulation side and this hook (pedestrian hung at frame 40): `node/carla_scenario_bridge/`
held `out.1` (the dying bridge, pid 519519: the hook firing at 07:43:27, `did not return
(waited 45.0 s, bound 45 s)` at 07:44:12, `Drained 1 request(s)`, `Exiting with status 75`),
`status.1` = 75, and `out`/`pid` of the respawned bridge (pid 534223, up 07:44:16). No
`tail -F` needed once the simulation side runs that play_launch.

*Step 3.* `budgets.rs`: frame wait 10 s, tick 45 s, `apply_settings` 45 s, `load_world` 150 s,
TM calls 45 s, `Initialize` deadline 280 s, spawn deadline 75 s; `enable_sync_mode`, the async
restore, `UpdateStepTime` and the map load run under the watchdog, each capped by the request's
deadline; `Initialize` fails naming the step when it is wedged or past its deadline. The 032
doc's "twice the frame wait" (30 + 20 = 50 s > 40 s) and the design doc (a Timeouts table, a
row for a hung call) corrected. T1 above ran this build.

*Step 4.* Spawns go through `apply_batch_sync`; `spawn_ladder` lifts only on CARLA's collision
error and adopts after a failed call. Test hook `CSB_TEST_SPAWN_TIMEOUT_MS` (every spawn RPC
times out after that many ms). Live with 1 ms (load ~30): every spawn of `ego_drive` and
`simulator_autopilot` failed as a call (`time-out of 1ms`) and was adopted (`CARLA created it
anyway as actor 9402 ... adopted, not spawned again`), both PASS, a census during each run
showed exactly one `hero`, none after. **The first live run found a bug**: the lookup ran on
csb's own client, whose never-evicted actor cache still held the previous scenario's
destroyed `hero` 9381 (the watermark had fallen below it once teardown emptied the ledger);
the second scenario adopted the dead actor and timed out. Fixed: the lookup goes through a
short-lived client (finding 7) and the watermark only rises (`highest_actor_id`).

*Step 5.* `actor_census`: a short-lived client scans ids 1..watermark + 16384 (8192 a call,
following higher finds); the reaper and `vehicle_occupying` use it; the reaper destroys by
server-side batch `DestroyActor`, children first. **The first live run found a second bug**
(finding 8): destroying the census client's actors with `Actor::destroy` was refused for the
props and walker ("already dead"), which stayed on the road, and the next scenarios' ego
stopped before the barrier (`ego_drive`, `engage_state` timed out). Fixed as above; the next
`Initialize` reaped the three leftovers. Live after T1: the respawned bridge's reaper logged
exactly the dead process's four actors -- `hero` 9434, `barrier_0` 9436, `barrier_1` 9437,
`pedestrian_0` 9438 -- no phantom, no "not found", nothing `csb_entity:` or `hero` left after
the runs.

*Acceptance* (sim side without hooks, the same ego stack, load 25-48): starters 6/6 PASS
(ego_drive 42 s, engage_state 45, pedestrian 128, rear_contact 25, simulator_autopilot 47,
traffic_light 184); incidents 5/5 PASS (i1 112 s, i2 81, i3 89, i4 122, i5 85). Step 6 (the
LibCarla rebuild) was not done then; see below.

**2026-10-16 -- step 6 done: upstream's TM fix is in carla-rust's prebuilt LibCarla, csb and
acb on carla 0.15.1; the stress run shows no hang (0 in 248,795 ticks, where stock 0.9.16
hung 12 times in ~323k).** The fix:

- *Fork.* `jerry73204/carla` branch `0.9.16-for-carla-rust` @ `5fa188e`: upstream CARLA
  `e7e076f` ("Removed TM mutex": in synchronous mode `TrafficManagerLocal::SynchronousTick`
  runs `Step()` inline instead of signalling the worker thread and waiting on
  `step_end_trigger`) cherry-picked onto 0.9.16, plus `5fa188e` initialising the
  `last_frame` member that the backport turned from a zeroed local of `Run()` into an
  uninitialised field.
- *Prebuilt.* GitHub release `carla-rust/0.9.16-4` on that fork,
  `libcarla_client.0.9.16-x86_64-unknown-linux-gnu.tar.zstd`, sha256 `a399d7d7b474293b
  dda6b723cd82e78b7e774349f2f0a816a244f65ba81cf357`.
- *carla-rust.* `84112ab` and `d184948` on `main` (the prebuilt index points carla-sys at
  0.9.16-4; the release bump); `carla` and `carla-sys` 0.15.1 on crates.io.
- *Here.* `carla = { version = "0.15.1", features = ["carla-0916"] }` in both workspaces
  (csb `Cargo.toml`, acb `Cargo.toml`), both `Cargo.lock`s at carla/carla-sys 0.15.1, built
  with no `CARLA_DIR`/`CARLA_VERSION` in the environment so carla-sys took the crates.io
  download path; both build directories hold the 0.9.16-4 tarball (sha256 above).

*Toolchain caveat.* 0.9.16-4 was built with the system clang 14; 0.9.16-3 and earlier with
UE 4.26's bundled clang 10. The C++ ABI the bridges link against is libstdc++ either way and
everything builds and passes below, but a crash inside `libcarla_client` that does not
reproduce on 0.9.16-3 should consider the compiler change first.

*Binary check.* In all three installed bridges (`install/carla_scenario_bridge/...`,
`install/acb_bridge/...` here, and acb's own `install/acb_bridge/...`) the disassembly of
`TrafficManagerLocal::SynchronousTick()` calls only `Parameters::GetSynchronousMode()` and
`TrafficManagerLocal::Step()`; the 0.9.16-3 builds called `condition_variable::wait`,
`notify_one` and the mutex pair there, and had no out-of-line `Step()` at all.

*Stress* (a second CARLA 0.9.16 server on port 3000, `tmstress` client: three slots, each
in a loop of synchronous TMs with two autopilot vehicles per TM and a tight `World::tick`
loop, CPU hogs pinned to the same cores, a tick that does not return in 30 s counted as a
hang): stock 0.9.16-3 hung **12 times in ~323k ticks over 45 min** (two runs); the patched
library **0 times in 248,795 ticks over 30 min** (~46 ticks/s a slot). gdb stacks of the
stock hangs match the lost wakeup this phase studied: the client in `SynchronousTick`
waiting on `step_end_trigger` (`TrafficManagerLocal.cpp:257`) while the worker waits on
`step_begin_trigger` (`:158`). Scratch: `/tmp/claude-1000005/-home-aeon-repos-carla-scenario-
bridge/f372598a-d329-483b-9e38-80a742b40cbd/scratchpad/s034_6` (`runs/`, `drive.sh`,
`patches/`, `tmstress/`).

*Regression* (the new build, CARLA 0.9.16 Epic Town01 (the same server), the simulation side
restarted after the build, one ego stack, the host loaded by other work on it (an nros cargo
build matrix, `FVP_BaseR_AEMv8`) and this repo's `just check` alongside): starters **6/6
PASS**, 01:46-01:59 UTC, 1/5/15-min load 95/128/142 at start and 125/124/132 at end (ego_drive 219 s -- the first
run on a stack just up, 42 s on 10-15 -- engage_state 47, pedestrian 132, rear_contact 27,
simulator_autopilot 49, traffic_light 187); incidents **5/5 PASS**, 02:07-02:17 UTC, load 96/100/117 at
start and 92/99/109 at end (i1 168 s, i2 86, i3 95,
i4 119, i5 76). `simulator_autopilot` ran the patched synchronous TM (`Traffic Manager on
port 8000: synchronous=true` at its spawn, shut down at its despawn); no hung call, drain or
restart in the bridge log, one bridge process throughout. `just test` 328 nextest + 174
pytest pass; `just check` clean. acb's own `just test` and
`just check` did not run to completion: `/home` filled to 0 bytes during them (another job's
build matrix on the same disk), failing the link with ENOSPC -- acb's build and the
bridges' scenarios above are its evidence for now; re-run both when the disk has room. Scratch:
`/tmp/claude-1000005/-home-aeon-repos-carla-scenario-bridge/f372598a-d329-483b-9e38-80a742b40cbd/scratchpad/s034_6b`
(`runs/starters`, `runs/incidents`, build and test logs).

