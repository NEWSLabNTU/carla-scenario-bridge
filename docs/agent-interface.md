# Writing an agent

An **agent** connects an autopilot to the scenarios run on carla-scenario-bridge. The
scenario simulator (SSv2) never talks to your autopilot directly: it talks to the **agent
relay** on the simulation side, and the relay talks to the agent registered under the
scenario entity's name (`ego`, `bg_av_1`, ...), over plain TCP. The agent turns the
relay's commands -- reset, drive to a goal, keep under a speed limit, stop -- into whatever
your autopilot understands, and reports back what the autopilot is doing.

Every agent is a Python class, `csb_agent.Agent`, in your own project. Nothing in this
workspace has to change, and the agent can run on another host than the simulator: it only
needs to reach the relay's port (default 5560).

```
 simulation side                                    vehicle side (any host)
 SSv2 ── carla-scenario-bridge ── CARLA              your autopilot
   └──── agent relay :5560 ◀──── TCP ────▶ your agent (csb_agent.Agent)
```

The Autoware vehicle side shipped with this project (`just ego-av`) is such an agent
already; this page is for driving with something else, or with Autoware in another way.

## Three ways to write one

| | When | You write |
|---|---|---|
| **Your own class** | any autopilot: a ROS stack, a learned policy, a simulator API, anything in Python | a subclass of `csb_agent.Agent` |
| **`ros_adapter` + YAML** | a ROS 2 autopilot whose steps are single topics, services and states | `agent.yaml` only |
| **A subclass of ours** | the YAML covers most of it but one step needs code | a `RosAdapter` subclass overriding that hook |

All three are `csb_agent.Agent` subclasses, and `csb-agent check` tests any of them.

## Your own class

Start from a skeleton that already passes the conformance check:

```bash
csb-agent new my-pilot          # or: ros2 run csb_agent csb-agent new my-pilot
cd my-pilot
csb-agent check my_pilot.agent:MyPilotAgent
```

`my_pilot/agent.py`:

```python
import csb_agent

class MyPilotAgent(csb_agent.Agent):
    def on_reset(self, ep):             # a scenario starts: forget the last one, get ready
        self.autopilot.relocalize(ep.pose)

    def on_goal(self, goal, waypoints): # drive to goal through waypoints
        self.autopilot.drive_to(goal, waypoints)
        self.report(csb_agent.DRIVING)

    def on_speed_limit(self, mps):      # None: no limit
        self.autopilot.max_speed = mps

    def on_stop(self):                  # stop, and hold until the next goal
        self.autopilot.stop()

if __name__ == "__main__":
    csb_agent.main(MyPilotAgent)
```

Run it, with the simulation side up (`just run`):

```bash
python -m my_pilot --relay tcp://sim-host:5560 --entity ego --config agent.yaml
```

### Hooks

Override the ones your autopilot can honour. A hook returns normally for success and
raises `csb_agent.Failed("why")` to fail the command; a hook you do not override is
answered "unsupported", and a scenario that needs it fails saying so. Hooks run one at a
time, on one thread; do not block in one for longer than its command allows (below).

| Hook | Called when | Must | Time allowed |
|---|---|---|---|
| `on_reset(ep)` | a scenario starts (SSv2 initializes the entity) | drop goal, plan and controller state; re-localize at `ep.pose`; return once ready for a goal | the relay's `reset_timeout` (60 s) |
| `on_goal(goal, waypoints)` | the scenario assigns a route | start planning and driving | 9 s |
| `on_speed_limit(mps)` | a speed limit is set or lifted (`None`) | apply it | 2.5 s |
| `on_stop()` | the scenario disengages the vehicle | stop, hold until the next goal | 2.5 s |
| `on_clear_goal()` | the route is cleared | drop the goal (default: `on_stop`) | 2.5 s |
| `on_episode_end(ep, result)` | `run_suite` reports the verdict (`result["passed"]`, `["failures"]`) | nothing; for logs, bags, learning | 2.5 s |
| `on_restart()` | a restart is asked for (capability `restart`) | restart the autopilot in place | -- |
| `on_cooperate(module, command, uuid)`, `on_cooperate_auto(module, enable)` | the scenario drives Autoware's RTC | -- | 2.5 s |
| `close()` | the agent shuts down | release what it holds | -- |

Poses are dictionaries `{x, y, z, qx, qy, qz, qw}` in the map frame (the same frame as the
scenario's Lanelet2 map). `ep` (`csb_agent.Episode`) carries `pose` and, from the relay,
an `episode` id; `scenario`, `map` and `parameters` are empty for now.

`self.config` is the `--config` file (JSON or YAML) as a dict, `self.entity` the entity
name, `self.log` a logger.

### Reporting state

The scenario reads your autopilot's state from what the agent reports. Call
`self.report(...)` from any thread whenever something changes; only the fields you pass
change:

```python
self.report(csb_agent.ARRIVED)                       # phase
self.report(pose={"x": ..., "y": ..., ...})          # the autopilot's own pose estimate
self.report(fault=csb_agent.MINIMAL_RISK_MANEUVER, fault_behavior="EMERGENCY_STOP")
self.report(detail="waiting for localization")       # free text, shown in the relay's log
```

| Phase | Means | Set by |
|---|---|---|
| `INITIALIZING` | not ready (re-localizing) | the SDK while `on_reset` runs |
| `IDLE` | ready for a goal | the SDK after `on_reset`, or you |
| `PLANNING` | goal received, no route yet | the SDK after `on_goal`, or you |
| `READY` | route set, about to drive | you |
| `DRIVING` | driving | **you** |
| `ARRIVED` | at the goal | **you** |
| `STOPPED` | stopped by command | the SDK after `on_stop` |

The SDK sets a phase after a hook only if the hook reported none. Reporting `ARRIVED` is
how a scenario's goal condition is met, so an agent that never reports it never finishes a
scenario that waits for arrival.

### What the SDK does for you

Registration with the relay, reconnecting when it goes away (every 1-5 s), state pushes at
5 Hz (which double as heartbeats), one command at a time with the state the command
produced sent before its answer, ignoring the scenario's retries of a command already done,
and talking to an older relay (protocol v1) when that is what it finds.

## `ros_adapter`: a ROS 2 autopilot from YAML

When each step of your autopilot is one topic or one service, no Python is needed:

```bash
ros2 run csb_agent ros_adapter --config agent.yaml
```

```yaml
entity: ego
relay: tcp://sim-host:5560
goal:        {publish: /planning/goal, type: geometry_msgs/msg/PoseStamped, from: goal}
speed_limit: {publish: /planning/max_velocity, type: std_msgs/msg/Float32, from: mps,
              "null": 1000.0}
stop:        {call: /autopilot/stop, type: std_srvs/srv/Trigger}
reset:       [{call: /autopilot/reset, type: std_srvs/srv/Trigger},
              {publish: /initialpose, type: geometry_msgs/msg/PoseWithCovarianceStamped,
               from: pose},
              {wait_phase: IDLE, timeout: 30}]
phase:       {subscribe: /autopilot/state, type: std_msgs/msg/String,
              map: {idle: IDLE, driving: DRIVING, arrived: ARRIVED, stopped: STOPPED}}
pose:        {subscribe: /localization/pose, type: geometry_msgs/msg/PoseStamped}
```

Each hook (`reset`, `goal`, `speed_limit`, `stop`, `clear_goal`, `episode_end`, `restart`)
is one action or a list run in order. A hook not in the file is unsupported; `reset` is
only offered to the relay when it is mapped.

| Action | Does |
|---|---|
| `publish: TOPIC, type: T` | publishes one message (reliable, transient local) |
| `call: SERVICE, type: T` | calls the service; fails if the response has a `success` field that is false; `timeout` (5 s) |
| `wait_phase: PHASE, timeout: S` | waits until the `phase` subscription reports PHASE (needs `phase:`) |
| `sleep: S` | waits |

Common keys: `fields: {...}` static values, as `ros2 topic pub` takes them; `from: pose |
goal | mps` the episode's start pose, the goal or the speed limit; `field: a.b` where it
goes (`name[]` appends to a sequence, e.g. `field: pose[]` for
`autoware_adapi_v1_msgs/srv/InitializeLocalization`); `"null": value` what to send when a
limit is lifted (without it, nothing is sent). `phase.field` names the subscribed message's
field (default `data`); `pose` takes `PoseStamped`, `PoseWithCovarianceStamped` or
`Odometry`.

### One step in code

Subclass `csb_agent.ros_adapter.RosAdapter`, override the hook, and run the configured
actions with your values:

```python
from csb_agent.ros_adapter import RosAdapter

class GoalAhead(RosAdapter):
    def on_goal(self, goal, waypoints):
        goal = {**goal, "x": goal["x"] + 10.0}           # your logic
        self.run_hook("goal", goal=goal, waypoints=waypoints)
```

Any class can get a ROS node the same way: mix in `csb_agent.ros.RosNode` and call
`self.start_node("name")`; `self.node` is spun on its own thread and stopped in `close()`.

## Checking an agent

`csb-agent check` runs your class against a fake relay -- no CARLA, no SSv2, no autopilot
beyond what your class starts -- through a scripted episode: register, reset, goal, speed
limit, stop, episode end, a second reset, a dropped connection. It prints each rule with
what it saw and exits non-zero on a failure:

```
$ csb-agent check my_pilot.agent:MyPilotAgent
  PASS  registers within 5 s
  ...
  FAIL  reset: IDLE before the reply -- phase INITIALIZING
  ...
FAIL: 14/16 rules
```

`--config agent.yaml` passes a config, `--reset-timeout S` (30) bounds the reset, `-v`
shows the SDK's log. A file works as well as a module: `csb-agent check agent.py:MyAgent`.
Passing the check means the agent follows the protocol; whether it drives well is the
scenarios' to say.

## Between scenarios: reset

At the start of every scenario the agent's `on_reset` runs, with the pose the scenario
places the vehicle at. The scenario simulator waits while the agent reports `INITIALIZING`,
and goes on once it reports `IDLE`, so a reset may take as long as the autopilot needs to
re-localize, up to the relay's `reset_timeout` (default 60 s). A reset that fails or runs
out of time fails that scenario.

Restarting the autopilot when a reset fails (`csb-agent supervise`, `run_suite
--restart=...`) is planned (roadmap 030 step 8).

## Compatibility

The protocol is versioned. Agents written with this SDK speak v2 and fall back to v1 on an
older relay; the relay serves v1 agents (such as the Autoware agent today) unchanged, giving
them the v1 sequence -- `stop`, `clear_goal`, `teleported(pose)` -- instead of `reset`. An
agent without `on_reset` is treated the same way by the SDK. The protocol itself, for
writing an agent in another language: [design/user-workflow.md, "The agent protocol
(I3)"](design/user-workflow.md#the-agent-protocol-i3).
