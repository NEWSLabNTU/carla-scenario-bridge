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
5. [ ] **CARLA reference adapter.** `carla_basic`; `basic/town01_ego_drive` passes with it
   driving and no Autoware running.
6. [ ] **Autoware adapter.** acb's agent moved onto the SDK (fork commit first), overridable
   steps; the starter `basic/` and `awf/` suites pass as before; acb's protocol copy removed.
7. [ ] **ROS adapter.** `ros_adapter` and its YAML schema, with a test autopilot node; a
   subclass overriding one step.
8. [ ] **Supervisor and restart ladder.** `csb-agent supervise`; `run_suite --restart`;
   result records; live: a reset made to fail (an agent that never reports ready) is
   restarted and the next scenario passes.
9. [ ] **User docs.** `docs/agent-interface.md` (the three ways, `check`, supervise,
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

