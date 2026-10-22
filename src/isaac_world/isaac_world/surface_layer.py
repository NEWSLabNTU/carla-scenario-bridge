"""The surface layer on Isaac (roadmap 047 S1): each ego wheel's friction, every physics step,
from the loaded world's surface map (`isaac_world.surfaces`). Import after `SimulationApp`.

After each substep the wheel's ground hit (aib's `Vehicle.contacts`) names a point and a
collider: the collider's SimReady label (`semantics:labels:wikidata_qcode` on it or an
ancestor) picks the label default, a zone over the point overrides it, and the friction for
the next substep goes into the wheel's own table (`Vehicle.set_wheel_friction`). A world
with no surface map is left alone: every wheel keeps the rig's tire friction.

Where the map comes from: `surfaces.yaml` in the map directory of a world file the server
was started with (`--world`), looked for at every `LoadWorld`; swb's `LoadWorld` sends only
the USD, which `register` maps back. (046's world protocol is the place for the map to say so
itself.)
"""
import logging
import os

from . import surfaces

log = logging.getLogger("isaac_world")

QCODE_ATTR = "semantics:labels:wikidata_qcode"


class SurfaceLayer:
    def __init__(self, stage, surface_map, map_origin=(0.0, 0.0, 0.0)):
        self.stage = stage
        self.map = surface_map
        self.origin = tuple(float(c) for c in map_origin)  # the world frame's origin in the map's
        self.condition = surfaces.RoadCondition()
        self.labels = {}  # collider path -> SimReady ground label, or None
        self.wheels = {}  # vehicle path -> surfaces.WheelSurfaces
        self.logged = {}  # (vehicle path, wheel) -> the surface last logged

    def label(self, path):
        """The ground label of the collider at `path`: the first Q-code found on it or up its
        ancestors, if it is a ground class."""
        if path in self.labels:
            return self.labels[path]
        label = None
        prim = self.stage.GetPrimAtPath(path) if path else None
        while prim and prim.IsValid() and not prim.IsPseudoRoot():
            attr = prim.GetAttribute(QCODE_ATTR)
            if attr and attr.HasAuthoredValue():
                label = next((surfaces.QCODE_LABELS[c] for c in (attr.Get() or [])
                              if c in surfaces.QCODE_LABELS), None)
                break
            prim = prim.GetParent()
        self.labels[path] = label
        return label

    def update(self, vehicle, speed, dt):
        """After a substep: each wheel of aib `vehicle` gets its friction for the next one."""
        if vehicle.contacts is None or vehicle.wheel_states is None:
            return
        ws = self.wheels.get(vehicle.path)
        if ws is None:
            ws = self.wheels[vehicle.path] = surfaces.WheelSurfaces(
                self.map, vehicle.p.tire_friction, vehicle.p.friction_vs_slip,
                len(vehicle.wheel_paths))
        ox, oy, _ = self.origin
        for i, (c, w) in enumerate(zip(vehicle.contacts, vehicle.wheel_states)):
            s = self.map.at(c.position[0] + ox, c.position[1] + oy, self.label(c.shape)) \
                if c.on_ground else None
            mu = ws.update(i, s, self.condition, speed, w.slip, dt)
            vehicle.set_wheel_friction(i, mu)
            if s is not None and self.logged.get((vehicle.path, i)) != s:
                # a wheel onto other ground: rare (class and zone edges), so INFO
                self.logged[(vehicle.path, i)] = s
                log.info("%s wheel %d at (%.1f, %.1f): %s%s%s%s, mu %.3f", vehicle.path, i,
                         c.position[0] + ox, c.position[1] + oy, s.cls,
                         f" {s.wetness}" if s.wetness else "",
                         "" if s.water_depth is None else f" water {s.water_depth * 1000:.1f} mm",
                         f" (zone {s.zone})" if s.zone else "", mu)

    def surfaces_under(self, vehicle):
        """The surface each wheel last stood on (None in the air), for logs and probes."""
        ws = self.wheels.get(vehicle.path)
        return list(ws.surfaces) if ws else []

    def reset(self, vehicle_path):
        """A teleport: the vehicle's filters snap to the ground on the next update."""
        ws = self.wheels.get(vehicle_path)
        if ws is not None:
            ws.reset()

    def forget(self, vehicle_path):
        self.wheels.pop(vehicle_path, None)


def register(world_files):
    """`{usd realpath: (surfaces.yaml path, map origin)}` for the world files' map
    directories, whether the surface map exists yet or not (one added later counts from the
    next LoadWorld); one that exists and fails its check stops the server here, with every
    problem listed."""
    import yaml

    out = {}
    for world_file in world_files:
        map_dir = os.path.dirname(os.path.dirname(world_file))
        with open(world_file) as f:
            w = yaml.safe_load(f) or {}
        usd = w.get("usd", "")
        if not usd:
            continue
        usd = usd if os.path.isabs(usd) else os.path.join(map_dir, usd)
        path = surfaces.find(map_dir)
        if path.exists():
            surfaces.load(path)  # refused here, not at the first LoadWorld
        out[os.path.realpath(usd)] = (str(path), tuple(w.get("map_origin") or (0.0, 0.0, 0.0)))
    return out
