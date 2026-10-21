# Phase 030: Agent API

Any autopilot can drive an SSv2 scenario in CARLA through one Python interface, written in
the user's own project, without touching this workspace. Between scenarios the agent is
**reset**; a **restart** of the autopilot is an option, used when a reset fails or by
policy.

Before this phase the relay's agent protocol (I3, [user-workflow.md](../design/user-workflow.md#the-agent-protocol-i3),
then `scenario_agent_relay/protocol.py`, now `csb_agent/protocol.py`) was generic, but only acb implemented it
(`acb_pilot/agent_core.py` with a copy of `protocol.py`), so changing how an autopilot is
driven means editing acb. The scenario boundary is implicit: SSv2's `initialize` reaches the
agent as `stop`, `clear_goal`, `teleported`, and the agent has no way to say "reset and
ready" other than a phase change; nothing tells it a scenario ended or how.

**Source**: user direction 2026-10-13 ("we should have a way for users to customize autopilot
interaction"; "reset over restart, keep restart as an option"; "a supervisor is OK";
"Python only for now"; "good to have a unified interface").
**Depends on**: [017](017-user-workflow.md) (relay, protocol v1, acb's agent),
[028](028-run-tooling.md) (`run_suite`'s error classes), [029](029-prebuilt-releases.md)
(the SDK ships in the release tarball).
**Takes over** 028's step 3 (the agent interface for users): its reference agent is this
phase's CARLA adapter, its user page this phase's guide.

## Design

### One interface: `csb_agent.Agent`

A Python class is the only interface the system knows. Everything that drives a vehicle --
our adapters, a user's class, the YAML-configured ROS adapter -- is an `Agent` subclass, and
the conformance check, the supervisor and the relay see nothing else.

```python
import csb_agent

class MyAgent(csb_agent.Agent):
    capabilities = {"reset"}                       # also: "restart", "cooperate"

    def on_reset(self, ep):           ...         # clear plan, re-localize at ep.pose; return when ready
    def on_goal(self, goal, waypoints): ...       # plan and drive; report DRIVING
    def on_speed_limit(self, mps):    ...
    def on_stop(self):                ...
    def on_clear_goal(self):          ...         # default: on_stop
    def on_episode_end(self, ep, result): ...     # optional: logs, bags, learning
    def on_restart(self):             ...         # only with the "restart" capability

if __name__ == "__main__":
    csb_agent.main(MyAgent)                        # --relay, --entity, --config; runs the loop
```

The SDK owns registration, heartbeats, reconnecting, command ordering (one command at a
time, the state the command produced pushed before its reply), timeouts, and the v1 fallback.
A hook returns normally for `OK`, raises `csb_agent.Failed(msg)` for `FAILED`; a hook not
overridden answers `UNSUPPORTED`. `self.report(phase=..., fault=..., detail=..., pose=...)`
pushes state; `ep` carries `episode` (id), `scenario`, `map`, `pose`, `parameters` (the
variant's, when SSv2 reports them).

**Package**: `src/csb_agent/` in this repository, pure Python, standard library only (ROS
optional), installed with the workspace and the release tarball (029) and published as a
wheel on each GitHub release for vehicle hosts without the tarball. `protocol.py` moves into
it; the relay imports it from there.

### Three ways to write an agent, all `Agent`

| | When | What the user writes |
|---|---|---|
| **Own class** | Any autopilot: non-ROS, a learned policy, a simulator API, logic in a step | A subclass, as above |
| **`ros_adapter` + YAML** | A ROS 2 autopilot whose steps are single topics / services / actions | `agent.yaml` only, run with `ros2 run csb_agent ros_adapter --config agent.yaml` |
| **Subclass one of ours** | Autoware, or YAML plus one step that needs code | A subclass overriding one hook or one step method |

`ros_adapter`'s YAML maps each hook to publishes, calls and sequences, and the phase to a
subscription with a value map; `pose` in a mapping is the episode's start pose:

```yaml
entity: ego
relay: tcp://sim-host:5560
goal:        {publish: /planning/goal, type: geometry_msgs/msg/PoseStamped}
speed_limit: {publish: /planning/max_velocity, type: std_msgs/msg/Float32}
stop:        {call: /autopilot/stop, type: std_srvs/srv/Trigger}
reset:       [{call: /autopilot/reset, type: std_srvs/srv/Trigger},
              {publish: /initialpose, from: pose},
              {wait_phase: IDLE, timeout: 30}]
phase:       {subscribe: /autopilot/state, type: std_msgs/msg/String,
              map: {idle: IDLE, driving: DRIVING, arrived: ARRIVED}}
```

The `csb_agent.ros` mixin gives any class an `rclpy` node spun in a thread (`self.node`),
which is what `ros_adapter` and the Autoware adapter are built on.

### Adapters we ship

- **Autoware** (`csb_agent.adapters.autoware.AutowareAgent`): today's `acb_pilot` agent
  (`agent_core.py`, `autoware_port.py`) moved onto the SDK, with its steps as overridable
  methods -- `relocalize(pose)`, `set_route(goal, waypoints)`, `engage()`,
  `wait_ready()` -- and its timeouts as parameters. acb's launch starts it when `relay:=`
  is given, as it starts `acb_agent` today; acb's copy of the protocol goes. acb is a fork
  submodule: its side is a fork commit, pushed before the pin moves.
- **ROS** (`ros_adapter`, above).
- **CARLA reference** (`csb_agent.adapters.carla_basic`): no ROS; finds the vehicle by role
  name, CARLA's `BasicAgent` route to the goal, target speed from the limit, brake on stop,
  re-plan on reset; control on every world tick (the bridge owns ticking). It is the
  example in the user guide and the agent of a starter run with no Autoware.

### Protocol v2

Additive to v1 (`VERSION` 2; a v1 agent on a v2 relay, and the reverse, work as today):

| Message | Direction | Fields | Meaning |
|---|---|---|---|
| `reset` | relay → agent (command) | `episode`, `scenario`, `map`, `pose`, `parameters` | a scenario boundary: drop goal, plan, controller state; re-localize at `pose`; reply `OK` only when ready for a goal (phase `IDLE`). Bounded by `reset_timeout`. Replaces `stop` + `clear_goal` + `teleported` for an agent with the `reset` capability; others get those three as today. |
| `episode_end` | relay → agent (event) | `episode`, `result` (`passed`, `failures[]`, `contacts`) | the scenario's verdict, from `run_suite` through the relay's commander role |
| `restart` | relay → agent (command), capability | -- | restart the autopilot in place; the agent re-registers when ready |

**Who sends `reset`**: the relay, on SSv2's `initialize` for the entity -- the same
trigger as today's `teleported`, so it works for every scenario launch, with or without
`run_suite`. `run_suite` adds `episode_end`, which only it can know.

### Reset first, restart as an option

Between scenarios the suite climbs one ladder, cheapest first:

1. **reset** -- always.
2. **restart** -- when the reset fails or times out, or by policy:
   `run_suite --restart=never|on-reset-failure|on-failure|every:N` (default
   `on-reset-failure`).
3. **infrastructure error** (028's class) -- when the restart fails; the suite moves on.

Each scenario's result records what it ran on (`reset`, `restarted`, the reset time), so a
result says how fresh its agent was.

A restart is done by the **supervisor**, or by the agent itself (`restart` capability) when
it has no supervisor:

```bash
csb-agent supervise --entity ego \
    --autopilot "ros2 launch my_stack bringup.launch.py" \
    --agent "python -m my_agent --config agent.yaml"
```

The supervisor starts both in one process group of its own, registers with the relay as
the entity's supervisor (a commander role), and on a restart request stops the group (TERM,
then KILL after a grace period, by process group, never by name) and starts it again;
ready is the agent's registration arriving at the relay. It survives a hung autopilot and
reaps what the group leaves. Without a supervisor and without the capability, `--restart`
other than `never` is refused at suite start, naming the entity.

### Tools

- `csb-agent new <name>` -- a project skeleton: `pyproject.toml` depending on `csb-agent`,
  the class with every hook stubbed, `agent.yaml`.
- `csb-agent check <module:Class> [--config agent.yaml]` -- conformance against a fake relay,
  no CARLA, SSv2 or Autoware: register, `reset`, `set_goal`, `set_speed_limit`, `stop`,
  `episode_end`, a second `reset`, a dropped connection; it reports each rule broken
  (missing reply, wrong phase after a command, reset never ready, missed heartbeat, v1
  fallback).
- `csb-agent supervise` -- above.

### Where a user's agent lives

In the user's own repository, anywhere, as a Python package or one script depending on
`csb-agent`; it runs where the autopilot runs (another host is fine) and reaches the relay
over TCP. Nothing is added to this workspace.

## Steps

1. [x] **Docs.** This phase doc; protocol v2 in user-workflow.md (normative), marked
   planned.
2. [x] **SDK core.** `csb_agent` package, `protocol.py` moved in (relay imports it),
   `Agent` and the run loop, v1 fallback; unit tests against the relay's server in-process.
3. [x] **Protocol v2 in the relay.** `reset` from SSv2's `initialize`, `reset_timeout`,
   `episode_end` via the commander role, mixed v1 / v2 tests.
4. [x] **`csb-agent check` and `new`.** The fake relay and its scripted episode; the
   skeleton passes `check` as generated.
5. [x] **CARLA reference adapter.** `carla_basic`; `basic/town01_ego_drive` passes with it
   driving and no Autoware running.
6. [x] **Autoware adapter.** acb's agent moved onto the SDK (fork commit first), overridable
   steps; the starter `basic/` and `awf/` suites pass as before; acb's protocol copy removed.
7. [x] **ROS adapter.** `ros_adapter` and its YAML schema, with a test autopilot node; a
   subclass overriding one step.
8. [x] **Supervisor and restart ladder.** `csb-agent supervise`; `run_suite --restart`;
   result records; live: a reset made to fail (an agent that never reports ready) is
   restarted and the next scenario passes.
9. [x] **User docs.** `docs/agent-interface.md` (the three ways, `check`, supervise,
   restart policy), the user guide and README pointing to it, a tutorial section running
   the CARLA reference agent.

## Acceptance

- One `Agent` interface: the Autoware, ROS and CARLA adapters are subclasses and pass
  `csb-agent check`.
- The starter suites pass with the Autoware adapter as before; `town01_ego_drive` passes
  with the CARLA adapter and no Autoware.
- Between scenarios the agent is reset (`reset` in v2, the v1 sequence for v1 agents);
  `--restart` restarts through the supervisor or the capability, and each result records
  which happened.
- A user writes an agent in their own project from `csb-agent new`, with the SDK from the
  release tarball or the wheel, and nothing else from this workspace.

### Results

**Step 2: SDK core (2026-10-21).** `src/csb_agent/` (ament_python, standard library only):

- `protocol.py`, moved from the relay, now v1 + v2: `VERSIONS = (1, 2)`, `encode(message,
  version)`, `decode` accepting either; `reset`, `episode_end`, `restart` with their
  argument checks, refused in a v1 message. The relay imports it from here.
- `Agent` (agent.py): the hooks of the design, `report()`, capabilities derived from the
  hooks a subclass overrides (`reset`, `restart`, RTC), `Episode`, `Failed`. A hook that
  reports no phase leaves the one its command implies (reset / teleported / clear_goal ->
  `IDLE`, set_goal -> `PLANNING`, stop -> `STOPPED`). `set_goal`'s extra arguments
  (`allow_goal_modification`, `segments`) reach `on_goal` only if its signature takes them.
- `Client` (client.py): registration, heartbeats via 5 Hz state pushes, reconnects (1 -> 5 s),
  one command at a time with the state pushed before the reply, a repeat of an identical
  `set_goal` / `teleported` / `reset` within 10 s of its success answered from memory, and
  the **v1 fallback** (a v2 register refused for its version -> v1 from then on; on v1 the
  boundary is `teleported`, which runs `on_reset` with only the pose).
- `csb_agent.main(MyAgent)`: `--relay` (or `$CSB_RELAY`), `--entity`, `--config` (JSON or
  YAML).
- Relay (`server.py`): **versions per connection**; each session is answered in its own
  version, a mid-connection version change is a protocol error, rejections go out in v1,
  and the relay answers `UNSUPPORTED` itself for a v2 command to a v1 agent or `reset` /
  `restart` to an agent without the capability. `AgentServer(versions=(1,))` stands in
  for a v1 relay in tests.

Tests: `csb_agent/test` (protocol incl. v1 / v2 round trips and v2-only refusals, agent and
dispatch without sockets) 93 pass; `scenario_agent_relay/test` 50 pass, including
`test_sdk_client.py` (the SDK against the real `AgentServer`: v2 registration and
capabilities, reset with the state before the reply, hooks, heartbeats over 4 s of
silence, v1 fallback against a v1-only relay, reconnect after a drop) and the mixed-version
server tests. `just test` now runs both directories (the relay's tests were not in it).
acb's unchanged v1 agent (`RelayClient` + `AgentCore` on its fake Autoware) registers with
the new relay as v1, answers `stop`, and is answered `UNSUPPORTED` for `reset` by the relay.
acb's drift test (its copy against the relay's `protocol.py`) now skips: the file it compares
with moved; step 6 deletes the copy.

**Step 3: protocol v2 in the relay (2026-10-21).**

- **`reset` on SSv2's `initialize`** (`scenario_agent_relay/episodes.py`, ROS-free): an
  agent with the `reset` capability gets one `reset` with a new episode id
  (`<entity>-<n>-<unix time>`); any other gets `teleported`, as in v1 (acb today).
  `scenario`, `map` and `parameters` go out empty: the relay is long-lived and learns
  neither from the concealer's `initialize`; filling them needs a source (SSv2's
  `/simulation/context`, or `run_suite` through the commander) -- open.
- **Bounded, and not held to the concealer's window**: the relay waits up to the
  concealer's 9 s for the agent's reply; past that it answers the concealer OK and lets the
  reset go on up to the node parameter `reset_timeout` (default 60 s). The SDK reports
  `INITIALIZING` while `on_reset` runs, which holds the concealer at "Autoware initializing"
  until the agent is `IDLE`. A repeated `initialize` for the same pose while the reset runs
  (the concealer's retry) joins it; nothing is sent twice.
- **How each reset went** (`episode`, `status`, `message`, `seconds`, `timed_out`) is kept
  per entity and returned to v2 commanders in `query` replies (`reset`), for step 8's
  ladder.
- **`episode_end`** goes through the commander role as any command does; the relay refuses
  it for a v1 agent (`UNSUPPORTED`). `csb_agent.Commander` is the client: `episode_end()`,
  `query()`, `command()`. Wiring it into `run_suite` is step 8, with the restart ladder.

Tests: `test_episodes.py` (8: a quick reset answered with its outcome, a failed one failing
the initialize, a slow one answered OK then completed with the concealer's retry joining it,
a reset past its timeout recorded, `teleported` for an agent without the capability, no
agent, the commander query carrying the last reset, `episode_end` through a `Commander`
reaching `on_episode_end` with the reset's episode). 151 pass across both packages. Not
yet run live: acb's agent is v1 and takes the unchanged `teleported` path; the first live
`reset` comes with step 5's CARLA adapter or step 6's Autoware adapter.

**Step 4: `csb-agent check` and `new` (2026-10-21).** One command, `csb-agent` (`ros2 run
csb_agent csb-agent`, or `python3 -m csb_agent`; a console script once installed from the
wheel):

- `check <module:Class | file.py:Class> [--config] [--reset-timeout 30]` runs the class
  with the SDK's `Client` against a fake relay on localhost (`csb_agent/check.py`) and a
  scripted episode: register, reset (or `teleported` for an agent without the capability),
  `set_goal`, `set_speed_limit`, `stop`, `episode_end` (v2), a second reset, a dropped
  connection. 16 rules, each printed PASS / FAIL with what was seen: registers in 5 s and
  offers every required command; speaks v2; pushes state after registering and at least
  every 1.5 s; each command answered in its window (reset: `--reset-timeout`; set_goal 9 s;
  others 2.5 s); the state before each reply (reset -> `IDLE`, set_goal -> past `IDLE`,
  stop -> `STOPPED`); registers again after the drop. Exit 1 on any failure.
- `new <name> [--dir]` writes `<name>/` with `pyproject.toml` (depends on `csb-agent`), the
  package with the agent class (every hook stubbed and commented), `__main__` running
  `csb_agent.main`, `agent.yaml` and a README; it passes `check` as generated (16/16).

Tests (`test_check.py`, 6): a good agent passes; one whose reset never reports ready fails
exactly the two "IDLE before the reply" rules; a 3.5 s stop fails its window; an agent
without reset is checked through `teleported` and passes; a generated project passes
`csb-agent check` through the CLI; a `file.py:Class` spec loads. SDK tests 99 pass.

**Step 7: ROS adapter (2026-10-21).** `csb_agent/ros.py`: the `RosNode` mixin (an rclpy
node on a multi-threaded executor in a daemon thread, `self.node`, stopped in `close()`)
and pose helpers for `Pose`, `PoseStamped`, `PoseWithCovariance(Stamped)` and `Odometry`.
`csb_agent/ros_adapter.py`: `RosAdapter`, every hook running the actions configured for
it (`publish`, `call`, `wait_phase`, `sleep`; `fields`, `from: pose | goal | mps`, `field`
with `name[]` appending to a sequence, `"null"` for a lifted limit), phase from a
subscription with a value map, pose from another; `reset` offered only when mapped; a
config that names no type, an unknown action or a `wait_phase` without a phase
subscription is refused at start. `ros2 run csb_agent ros_adapter --config agent.yaml`.
Only these two modules import ROS; the SDK core stays standard library.

New in the SDK on the way: `Agent.close()` (called by `csb_agent.main` at shutdown and by
`check` at its end; `RosNode` stops its node there), and only *phase* reports suppress a
hook's default phase -- a pose stream used to count as a report and skipped them.

Tests (`test_ros_adapter.py`, 5, in ROS domain 77 on localhost so they never meet a live
stack): a test autopilot node (goal topic, limit topic, `/initialpose`, reset and stop
services, a String state) driven by YAML only passes `csb-agent check` and received the
goal, the limit and both resets' poses; `reset` is not offered when unmapped; a lifted limit
publishes the `"null"` value; a subclass overriding `on_goal` (the goal 10 m ahead, the
rest from the YAML) passes the check and sent its own goal; three bad configs are refused.
SDK tests 106 pass.

**Step 9 (in part, 2026-10-21).** `docs/agent-interface.md`: the three ways, the hooks
with what each must do and its time, reporting state and the phases the SDK sets, what the
SDK handles, `ros_adapter`'s YAML, one step in code, `csb-agent check` / `new`, reset
between scenarios, compatibility (v1, the Autoware agent). Linked from the README, the
user guide ("Another autopilot", section 4) and `docs/README.md`. Still to come with their
steps: the tutorial section running the CARLA reference agent (step 5) and the restart
policy and supervisor (step 8).

**Plan for steps 5, 6 and 8 (2026-10-21).**

- *Step 5, `csb_agent.adapters.carla_basic`.* `CarlaBasicAgent`: connects to CARLA
  (`carla_host`, `carla_port`), finds the vehicle whose `role_name` is the bridge's ego
  role (`hero`) at each reset (a new actor per scenario), and drives it with CARLA's own
  `agents.navigation.BasicAgent` (PythonAPI from `carla_pythonapi`, else
  `$CARLA_DIR/PythonAPI/carla`; it needs `shapely` and `networkx`): `on_goal` traces the
  route through the waypoints with BasicAgent's global planner, `on_speed_limit` sets its
  target speed, `on_stop` brakes and holds. Control runs on world ticks: a `world.on_tick`
  callback only wakes a control thread (a slow callback would hold a synchronous server's
  frame), which runs `run_step()` and applies the ego's control. It never ticks, never
  changes settings, never spawns or destroys, and touches no actor but the ego; when the
  server stops answering (an idle probe), the process exits -- a CARLA client must not
  outlive its server (CLAUDE.md). `csb-agent check` runs it with `world: fake`: the same
  class over a kinematic stand-in for the world, so the check needs no CARLA (the check
  tests the protocol, not the driving). Live: `basic/town01_ego_drive` through `run_suite`
  with this agent registered as `ego` and no Autoware.
- *Step 6, `csb_agent.adapters.autoware.AutowareAgent`* (in the SDK, beside `ros_adapter`:
  ADAPI is Autoware's, not CARLA's, and any Autoware vehicle side can use it). acb's
  `AgentCore` and `AutowarePort` move in as the class and its ROS port; the steps are
  methods a subclass overrides -- `wait_ready()`, `relocalize(pose)`, `set_route(...)`,
  `engage()` -- and the bounds (`scan_wait`, `converge_timeout`, `initialize_attempts`,
  `engage_retry`, `settle`, `call_timeout`) are config keys. `on_reset` (v2) drops the
  route, stops and relocalizes; the v1 `teleported` path comes from the SDK. acb's
  `acb_pilot agent` keeps its executable name and parameters (the launch is unchanged) and
  becomes a thin entry over the class (relay mode through `csb_agent`'s client, local mode
  driving the same class to a goal file); acb's `agent_core.py`, `agent_protocol.py`,
  `autoware_port.py` and their tests go (the fake-Autoware tests move to the SDK). acb's
  agent then needs `csb_agent` installed (this workspace, the release tarball or the wheel).
  Fork commit pushed before the pin. Live: the starter `basic/` and `awf/` suites through
  `run_suite`, and a reset longer than the concealer's 9 s.
- *Step 8.* `csb-agent supervise --entity E --autopilot CMD --agent CMD [--relay]
  [--grace 10]`: starts both commands in one new process group, registers with the relay
  as a commander and answers restart requests addressed to the entity's supervisor by
  stopping the group (TERM, KILL after `--grace`, by pgid) and starting it again; ready is
  the entity's agent registering at the relay again. `run_suite --restart=never |
  on-reset-failure | on-failure | every:N` (default `on-reset-failure`): after each
  scenario it reads the entity's last reset from the relay (`query`), sends `episode_end`
  with the verdict through `csb_agent.Commander`, and restarts per policy before the next
  one; a non-`never` policy without a supervisor or the agent's `restart` capability is
  refused at start. Each scenario's result (summary and `suite.json`) records `reset`
  status and seconds, and `restarted`. Live: an agent whose reset never reports ready is
  restarted by the supervisor and the next scenario passes.

**Step 5: CARLA reference adapter (2026-10-21).** `csb_agent/adapters/carla_basic.py`,
`CarlaBasicAgent`, as planned: CARLA's `BasicAgent` (PythonAPI `agents`; `shapely` installed
for it on this host), vehicle by `role_name` at every reset, route traced through the
waypoints with BasicAgent's global planner, the limit as target speed, brake-and-hold on
stop, control on world ticks through a woken control thread, exit 3 when an idle probe
finds the server gone; pose reported in the map frame, moved to the scenario's vehicle
origin by the offset measured at the reset. `python3 -m csb_agent.adapters.carla_basic`.

- `csb-agent check csb_agent.adapters.carla_basic:CarlaBasicAgent` with `world: fake`:
  **16/16**. Tests (`test_carla_basic.py`, 4): the check, a drive through a waypoint to
  `ARRIVED`, speed limit and hold on stop, the frame helper. SDK tests 110 pass.
- **Live, no Autoware** (ego stack down; sim side on the current build, ClearNoon): the
  agent registered as `ego` (v2), `run_suite basic/town01_ego_drive.xosc`: **PASS** (20 s);
  a second agent process, `town01_ego_drive` + `town01_traffic_light`: **2/2 PASS** (20 s,
  167 s; BasicAgent waits at the red light). Each scenario's `reset` found the new vehicle
  (actors 2985, 2987, 2989) in under 0.6 s; set_goal -> ARRIVED in 9 s on ego_drive. The
  first run logged three pose warnings before its first reset, with no vehicle yet (fixed:
  no pose is reported without one).
- Tutorial: `docs/agent-interface.md`, "Tutorial: a scenario driven by CARLA's BasicAgent".

**Step 6: Autoware adapter (2026-10-21).** `csb_agent/adapters/autoware.py` (ROS-free) and
`autoware_ros.py` (the ADAPI port), as planned. `AutowareAgent` carries acb's core as hooks
and steps: `wait_ready(timeout)`, `relocalize(pose)` (fresh scans -> initialize -> converge,
blocking, `Failed` with the reason), `set_route(...)`, `engage()`; a background step (10 Hz)
engages a ready goal and reports Autoware's state (phase, MRM / emergency faults, turn
indicators, RTC, pose); `on_reset` reports INITIALIZING throughout, whatever Autoware shows,
then the derived phase. RTC is offered only when Autoware's RTC services answer. Bounds are
config keys (`DEFAULTS`). `ros2 run csb_agent autoware_agent`.

- acb `25470e4` (pushed before the pin): `acb_pilot agent` keeps its name and parameters
  and runs the class (relay mode through `csb_agent.Client`, local mode driving it to the
  goal file), plus `agent_config:=` (a settings file); `agent_core.py`, `agent_protocol.py`,
  `autoware_port.py` and their three test files deleted (the drift test with them). acb's
  agent now needs `csb_agent` (this workspace, the tarball or the wheel); acb's own workspace
  alone no longer runs it.
- Tests (`test_autoware_agent.py`, 23, on the moved fake Autoware and clock): the phase
  table; reset -> goal -> engage -> arrive; a reset drops a driven route first; fresh scans
  waited for but not forever; no convergence and refused initializes fail the reset with
  the reason; a refused initialize retried; services never answering; an overridden
  `relocalize`; goals (duplicate, reroute while driving, refused), stop holding, engage
  retries, unavailable, speed limit, RTC only with the services, faults, state validity;
  and `csb-agent check` on a fake Autoware: 16/16. SDK + relay tests 191 pass.
- **Live** (`just ego-av`, default clustering stack, the new agent; sim side on the current
  build): the agent registers as v2 with `reset`; starter suites through `run_suite`:
  **basic/ (six Town01) 6/6** (ego_drive 46 s, engage_state 43, pedestrian 125, rear_contact
  24, simulator_autopilot 45, traffic_light 183), **UC-AEB-001 9/9** (538 s), **UC-ACC-001
  27/27** (1967 s; a first run with `--timeout-per-scenario 600` was cut at 25 variants by that
  timeout, not a failure). Every scenario reset through v2: `ego: reset ego-<n>-<t> -> OK in
  4.3-5.8 s` in the relay's log, `localized at the reset pose after ...` in the agent's.
- **A reset longer than the concealer's 9 s** (`agent_config:=` with `scans_required: 100000,
  scan_wait: 14`, so each reset waits 14 s for scans): the relay answered the concealer at
  9 s (`initialize -> OK: reset ego-52-... in progress`), the reset completed in 16.6 s and
  17.7 s, and `town01_ego_drive` and `town01_traffic_light` passed (56 s, 197 s).
- Docs: `agent-interface.md` "Autoware: AutowareAgent"; user-workflow.md no longer
  describes acb's own copy.

**Step 8: supervisor and restart ladder (2026-10-21).**

- Protocol (v2): a third role, `supervisor` (`register_supervisor(entity)`; refused in a v1
  message); query replies to v2 commanders carry `supervised`. Relay: one supervisor
  session per entity beside the agent's (heartbeats, peer timeout), and `restart` for an
  entity goes to its agent when the agent offers it, else to its supervisor (no agent need
  be registered), else `UNSUPPORTED` naming both.
- `csb-agent supervise` (`csb_agent/supervise.py`): `--entity`, `--agent`, `--autopilot`
  (optional), `--relay`, `--grace` (10 s), `--respawn-delay` (10 s). Both commands in one
  new process group (the first leads it, the second joins it); stop = SIGTERM to the group,
  SIGKILL after the grace, reap, by pgid only. Answers `restart` once the new group runs;
  restarts the group when the agent exits on its own; stops it on its own exit.
- `run_suite --restart=never|on-reset-failure|on-failure|every:N` (default
  `on-reset-failure` when the relay reports a supervisor or the agent's `restart`
  capability, else `never` with a note; an explicit non-`never` policy without either is
  refused at start, exit 2) and `--restart-timeout` (300 s). After each scenario: the
  reset record from the relay (kept only when its episode is new), `episode_end` with the
  verdict through `csb_agent.Commander`, the ladder, then a wait for the agent to register
  again (phase past `UNAVAILABLE`). Results: the summary line (`[reset OK 0.5 s]`,
  `[restarted agent]`) and a new `<results>/suite.json` (per scenario: reset, restarted,
  the restart done after it). Deviation from the design: the default degrades to `never`
  rather than refusing, so a plain `run_suite` on a stack with no supervisor keeps working.
- Tests: `test_supervisor.py` (4: restart routed to the supervisor, old group gone by pgid,
  the agent back; an agent that exits started again; no supervisor and no capability ->
  `UNSUPPORTED`; the role needs v2) and run_suite's `RestartLadder` (6, on a fake
  commander: policy parsing, refusal, a failed reset restarting and marking the next
  scenario, a stale reset not recorded, every:N and on-failure, a restart that never
  registers failing). csb_launch 23 + SDK/relay tests pass.
- **Live** (CARLA reference agent, no Autoware): `csb-agent supervise --entity ego --grace 5
  --agent "never_ready_once.py ..."` -- a CarlaBasicAgent whose first reset blocks for good
  while a marker file exists (scratch `s_4d/a8`). `run_suite <two copies of
  town01_ego_drive> --restart on-reset-failure`: the first scenario's reset timed out at the
  relay's 60 s (`reset ego-1-... -> FAILED in 60.0 s`) and SSv2 failed it ("waited for ...
  WAITING_FOR_ROUTE ... INITIALIZING"); run_suite asked for a restart, the supervisor
  stopped group 1108743 and started 1121799 (`restart OK in 2 s`), the new agent registered,
  and the second scenario **passed** on it (`[reset OK 0.5 s] [restarted agent]`);
  `suite.json` records both.

**Step 9 (rest, 2026-10-21).** `agent-interface.md`: the CARLA reference agent tutorial
(step 5), "Autoware: AutowareAgent" (step 6), "When a reset fails: restart" and
"`csb-agent supervise`" (this step).
