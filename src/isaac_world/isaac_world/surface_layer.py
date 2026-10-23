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

In a soil zone (047 S3, `soil: grid | closed_form`) each wheel also runs the soil layer
(`isaac_world.soil`) on the load PhysX's suspension reports: the soil-limited friction replaces
the class's in the wheel's table, and the soil's resistance pushes back on the body at the
contact, for the next substep (`apply_force`, the host's `apply_force_at_pos`). Both fade in
over `SOIL_BLEND` wheel diameters from the zone's edge. The grid's ruts are fresh at every
`fresh_soil` (a new owner or a release).
"""
import logging
import math
import os

from . import soil, surfaces

log = logging.getLogger("isaac_world")

QCODE_ATTR = "semantics:labels:wikidata_qcode"
# The soil's share grows from 0 on a soil zone's edge to 1 this many wheel diameters inside.
SOIL_BLEND = 1.5
# Node spacing of a soil zone's grid, m.
SOIL_SPACING = 0.05


class SurfaceLayer:
    def __init__(self, stage, surface_map, map_origin=(0.0, 0.0, 0.0), apply_force=None):
        """`apply_force(body path, force, position)`, world frame, N and m: a force on a body
        for the next physics step (the soil's resistance); None: the soil sets friction only."""
        self.stage = stage
        self.base = surface_map  # the map as loaded; `map` is it under the road condition
        self.map = surface_map
        self.origin = tuple(float(c) for c in map_origin)  # the world frame's origin in the map's
        self.condition = surfaces.RoadCondition()
        self.labels = {}  # collider path -> SimReady ground label, or None
        self.wheels = {}  # vehicle path -> surfaces.WheelSurfaces
        self.logged = {}  # (vehicle path, wheel) -> the surface last logged
        self.apply_force = apply_force
        self.grids = {}  # soil zone id -> soil.SoilGrid, made when a wheel first enters
        self.soil_out = {}  # (vehicle path, wheel) -> (soil.WheelSoil, share) of the last step

    def set_condition(self, friction_scale=1.0, wetness="dry", properties=None):
        """A scenario's road condition (OpenSCENARIO `RoadCondition`, 047 S2), from the next
        step on: the whole road state, replacing the last. SurfaceError lists every problem
        and changes nothing."""
        self.condition, self.map = surfaces.road_condition(self.base, friction_scale, wetness,
                                                            properties)
        zones = {name: value for name, value in sorted((properties or {}).items())
                 if name.startswith(surfaces.PROPERTY_PREFIX)}
        log.info("road condition: friction scale %g, %s%s", self.condition.friction_scale,
                 self.condition.wetness, "".join(f", {k}={v}" for k, v in zones.items()))

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
            target = None
            if s is not None and s.soil:
                target = self._soil(vehicle, i, c, w, s, speed, dt)
            else:
                self.soil_out.pop((vehicle.path, i), None)
            mu = ws.update(i, s, self.condition, speed, w.slip, dt, target)
            vehicle.set_wheel_friction(i, mu)
            if s is not None and self.logged.get((vehicle.path, i)) != s:
                # a wheel onto other ground: rare (class and zone edges), so INFO
                self.logged[(vehicle.path, i)] = s
                log.info("%s wheel %d at (%.1f, %.1f): %s%s%s%s, mu %.3f", vehicle.path, i,
                         c.position[0] + ox, c.position[1] + oy, s.cls,
                         f" {s.wetness}" if s.wetness else "",
                         "" if s.water_depth is None else f" water {s.water_depth * 1000:.1f} mm",
                         f" (zone {s.zone})" if s.zone else "", mu)

    def begin_step(self):
        """Once per physics step, before the vehicles' updates."""
        for g in self.grids.values():
            g.begin_step()

    def _soil(self, vehicle, i, contact, wheel, s, speed, dt):
        """Wheel `i` in soil zone `s.zone`: its resistance force for the next step, and its
        friction target relative to dry reference ground (blended toward the class's at the
        zone's edge)."""
        table = self.map.table
        zone = next(z for z in self.map.zones if z.id == s.soil_zone)
        params = table[s.cls].soil
        r, b = vehicle.p.wheel_radius, vehicle.p.wheel_width
        ox, oy, _ = self.origin
        x, y = contact.position[0] + ox, contact.position[1] + oy
        dx, dy, dz = contact.direction
        spin = wheel.rotation_speed * r
        if s.soil == "grid":
            grid = self.grids.get(zone.id)
            if grid is None:
                grid = self.grids[zone.id] = soil.SoilGrid(params, SOIL_SPACING)
                grid.begin_step()
                log.info("soil zone %s (%s): grid at %g m", zone.id, s.cls, SOIL_SPACING)
            out = grid.wheel((vehicle.path, i), x, y, math.atan2(dy, dx), contact.load, r, b,
                             spin, speed, dt)
        else:
            out = soil.wong_reece(params, contact.load, r, b, spin, speed)
        share = soil.edge_weight(zone.polygon, x, y, SOIL_BLEND * 2 * r)
        self.soil_out[(vehicle.path, i)] = (out, share)
        if self.apply_force is not None and out.resistance > 0 and share > 0:
            f = -out.resistance * share * (1.0 if speed >= 0 else -1.0)
            self.apply_force(vehicle.path, (f * dx, f * dy, f * dz), contact.position)
        soil_mu = self.condition.friction_scale * out.mu(contact.load) / table[table.reference].mu
        return share * soil_mu + (1 - share) * surfaces.peak_mu(table, s, self.condition, speed)

    def soil_under(self, vehicle):
        """Each wheel's (soil.WheelSoil, share) of the last step, None off soil."""
        return [self.soil_out.get((vehicle.path, i)) for i in range(len(vehicle.wheel_paths))]

    def fresh_soil(self):
        """Every soil zone back to untouched ground (no ruts)."""
        self.grids = {}
        self.soil_out = {}

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
        self.soil_out = {k: v for k, v in self.soil_out.items() if k[0] != vehicle_path}


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
