"""`ros_adapter`: an agent for a ROS 2 autopilot, configured by YAML instead of code.

    ros2 run csb_agent ros_adapter --config agent.yaml

Each hook maps to one action or a list of them, run in order; a hook not in the file is
answered UNSUPPORTED (and `reset` is only offered when mapped). The autopilot's phase comes
from a subscription with a value map; its pose, optionally, from another.

    entity: ego
    relay: tcp://sim-host:5560
    goal:        {publish: /planning/goal, type: geometry_msgs/msg/PoseStamped, from: goal}
    speed_limit: {publish: /planning/max_velocity, type: std_msgs/msg/Float32, from: mps}
    stop:        {call: /autopilot/stop, type: std_srvs/srv/Trigger}
    reset:       [{call: /autopilot/reset, type: std_srvs/srv/Trigger},
                  {publish: /initialpose, type: geometry_msgs/msg/PoseWithCovarianceStamped,
                   from: pose},
                  {wait_phase: IDLE, timeout: 30}]
    phase:       {subscribe: /autopilot/state, type: std_msgs/msg/String,
                  map: {idle: IDLE, driving: DRIVING, arrived: ARRIVED}}
    pose:        {subscribe: /localization/pose, type: geometry_msgs/msg/PoseStamped}

Actions:
  publish: TOPIC, type: T          publish one message (latched QoS: transient local)
  call: SERVICE, type: T           call; FAILED if it has a `success` field that is false
  wait_phase: PHASE, timeout: S    wait for the subscribed phase (default 30 s)
  sleep: S
  common keys: fields: {...} (static values, as `ros2 topic pub` takes them),
               from: pose | goal | mps (the episode's start pose, the goal, the limit),
               field: dotted.path (where `from` goes; `name[]` appends to a sequence;
               default: the message itself for a pose, `data` for mps),
               "null": value (for mps when the limit is lifted; default: skip the action)

Hooks: reset, goal, speed_limit, stop, clear_goal, episode_end, restart.

To change one step in code, subclass `RosAdapter` and override that hook, calling
`self.run_hook("goal", goal=...)` for the configured part.
"""

import re
import threading
import time

import csb_agent
from csb_agent import protocol as P

from . import ros as R

HOOKS = ("reset", "goal", "speed_limit", "stop", "clear_goal", "episode_end", "restart")
_HOOK_CAPABILITY = {"reset": P.RESET, "restart": P.RESTART}
_SEQ = re.compile(r"sequence<([\w/]+)(?:,\s*\d+)?>")
LATCHED_DEPTH = 1


class ConfigError(ValueError):
    pass


class RosAdapter(R.RosNode, csb_agent.Agent):
    """An agent whose hooks are the actions in its config."""

    name = "ros_adapter"

    def __init__(self, config=None):
        super().__init__(config)
        self.actions = {hook: self._actions(hook) for hook in HOOKS if hook in self.config}
        self._phase_value = None
        self._phase_seen = threading.Condition()
        self._publishers, self._clients = {}, {}
        self.start_node(self.config.get("node_name", f"csb_agent_{self.entity}"))
        self._subscribe()

    # -- configuration --------------------------------------------------------------------------
    def _actions(self, hook):
        spec = self.config[hook]
        actions = spec if isinstance(spec, list) else [spec]
        for action in actions:
            if not isinstance(action, dict) or not (
                    {"publish", "call", "wait_phase", "sleep"} & action.keys()):
                raise ConfigError(f"{hook}: each action is publish, call, wait_phase or "
                                  f"sleep (got {action!r})")
            if ("publish" in action or "call" in action) and "type" not in action:
                raise ConfigError(f"{hook}: {action!r} needs a type")
            if action.get("wait_phase") is not None and "phase" not in self.config:
                raise ConfigError(f"{hook}: wait_phase needs a `phase` subscription")
            if action.get("wait_phase") is not None and action["wait_phase"] not in P.PHASES:
                raise ConfigError(f"{hook}: unknown phase {action['wait_phase']!r}")
        return actions

    def capabilities(self):
        caps = list(P.REQUIRED_COMMANDS)
        caps += [cap for hook, cap in _HOOK_CAPABILITY.items() if hook in self.actions]
        return caps

    def _subscribe(self):
        phase = self.config.get("phase")
        if phase:
            mapping = {str(k): v for k, v in phase.get("map", {}).items()}
            bad = [v for v in mapping.values() if v not in P.PHASES]
            if bad:
                raise ConfigError(f"phase.map: unknown phases {bad}")
            field = phase.get("field", "data")

            def on_phase(msg):
                parent, attr = R.field_path(msg, field)
                value = str(getattr(parent, attr) if parent is not None else attr)
                mapped = mapping.get(value)
                if mapped is None:
                    return
                with self._phase_seen:
                    changed = mapped != self._phase_value
                    self._phase_value = mapped
                    self._phase_seen.notify_all()
                if changed:
                    self.report(mapped, detail=f"{phase['subscribe']}: {value}")

            self.node.create_subscription(R.message_type(phase["type"]), phase["subscribe"],
                                          on_phase, 10)
        pose = self.config.get("pose")
        if pose:
            self.node.create_subscription(
                R.message_type(pose["type"]), pose["subscribe"],
                lambda msg: self.report(pose=R.get_pose(msg)), 10)

    # -- hooks ----------------------------------------------------------------------------------
    def run_hook(self, hook, **values):
        """Run the configured actions of `hook` with `values` (pose, goal, mps)."""
        if hook not in self.actions:
            raise NotImplementedError
        for action in self.actions[hook]:
            self._run(hook, action, values)

    def on_reset(self, ep):
        self.run_hook("reset", pose=ep.pose)

    def on_goal(self, goal, waypoints):
        self.run_hook("goal", goal=goal, waypoints=waypoints)

    def on_speed_limit(self, mps):
        self.run_hook("speed_limit", mps=mps)

    def on_stop(self):
        self.run_hook("stop")

    def on_clear_goal(self):
        if "clear_goal" in self.actions:
            return self.run_hook("clear_goal")
        return self.run_hook("stop")

    def on_episode_end(self, ep, result):
        if "episode_end" in self.actions:
            self.run_hook("episode_end")

    def on_restart(self):
        self.run_hook("restart")

    # -- actions --------------------------------------------------------------------------------
    def _run(self, hook, action, values):
        if "sleep" in action:
            time.sleep(float(action["sleep"]))
            return
        if action.get("wait_phase") is not None:
            want, timeout = action["wait_phase"], float(action.get("timeout", 30.0))
            with self._phase_seen:
                if not self._phase_seen.wait_for(lambda: self._phase_value == want, timeout):
                    raise csb_agent.Failed(f"{hook}: the autopilot did not report {want} "
                                           f"within {timeout:.0f} s (last: "
                                           f"{self._phase_value})")
            return
        kind = "publish" if "publish" in action else "call"
        if kind == "publish":
            msg = R.message_type(action["type"])()
        else:
            msg = R.service_type(action["type"]).Request()
        R.fill(msg, action.get("fields"))
        if action.get("from") and not self._apply(msg, action, values):
            return  # nothing to send (a lifted limit with no "null" value)
        if kind == "publish":
            self._publisher(action).publish(msg)
            return
        client = self._client(action)
        timeout = float(action.get("timeout", 5.0))
        if not client.wait_for_service(timeout_sec=timeout):
            raise csb_agent.Failed(f"{hook}: service {action['call']} not available")
        future = client.call_async(msg)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise csb_agent.Failed(f"{hook}: {action['call']} did not answer in {timeout} s")
        response = future.result()
        if getattr(response, "success", True) is False:
            text = getattr(response, "message", "")
            raise csb_agent.Failed(f"{hook}: {action['call']} failed" +
                                   (f": {text}" if text else ""))

    def _apply(self, msg, action, values):
        source = action["from"]
        if source not in ("pose", "goal", "mps"):
            raise ConfigError(f"from: {source!r} (pose, goal or mps)")
        value = values.get(source)
        if source == "mps":
            if value is None:
                if "null" not in action:
                    return False
                value = action["null"]
            parent, attr = R.field_path(msg, action.get("field", "data"))
            setattr(parent, attr, type(getattr(parent, attr))(value))
            return True
        if value is None:
            raise csb_agent.Failed(f"no {source} to send")
        path = action.get("field", "")
        if path.endswith("[]"):
            parent, attr = R.field_path(msg, path[:-2])
            container = parent if parent is not None else msg
            types = container.get_fields_and_field_types()
            match = _SEQ.fullmatch(types[attr])
            if not match:
                raise ConfigError(f"field {path}: {attr} is not a sequence")
            element = R.message_type(match.group(1).replace("/", "/msg/", 1)
                                     if "/msg/" not in match.group(1) else match.group(1))()
            R.set_pose(element, value)
            getattr(container, attr).append(element)
            return True
        parent, attr = R.field_path(msg, path)
        R.set_pose(getattr(parent, attr) if parent is not None else attr, value)
        return True

    def _publisher(self, action):
        topic = action["publish"]
        if topic not in self._publishers:
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            qos = QoSProfile(depth=LATCHED_DEPTH, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._publishers[topic] = self.node.create_publisher(
                R.message_type(action["type"]), topic, qos)
        return self._publishers[topic]

    def _client(self, action):
        name = action["call"]
        if name not in self._clients:
            self._clients[name] = self.node.create_client(R.service_type(action["type"]),
                                                          name)
        return self._clients[name]


def main(argv=None):
    return csb_agent.main(RosAdapter, argv)


if __name__ == "__main__":
    main()
