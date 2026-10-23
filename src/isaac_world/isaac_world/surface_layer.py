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
over `SOIL_BLEND` wheel diameters from the zone's edge. A zone with `ground_z` has the
server's own ground (`soil_tiles`), into which `flush` puts the grid's ruts once a frame,
where no wheel stands. The
ruts are fresh at every `fresh_soil` (a new owner or a release).

Every vehicle shares a zone's grid (047 S4). The egos run on it as above; a kinematic vehicle
(an NPC SSv2 moves by pose) rolls on it without slip, its pose interpolated over the frame's
substeps (`set_kinematic`, `roll_kinematic`), so its ruts are the egos' ground; it feels
nothing back. Each substep updates the egos in handle order, then the kinematic vehicles in
path order, so two runs of a scenario cut the same ruts.
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
# A kinematic vehicle's wheels where its catalogue entry gives none (047 S4): wheel base and
# tread as shares of its box, CARLA's mid-size car's radius and a 0.2 m tire.
KINEMATIC_BASE, KINEMATIC_TREAD = 0.6, 0.85
KINEMATIC_RADIUS, KINEMATIC_WIDTH = 0.33, 0.2
KINEMATIC_JUMP = 5.0  # m in one frame: a teleport (100 m/s at 20 Hz)
G = 9.81


def kinematic_wheels(dims, center, physics=None):
    """A kinematic vehicle's wheels: ([(x, y) in the body frame, x forward], radius, width).
    CARLA's wheel base, tread and radius (`physics`, the catalogue's entry) when it gives them,
    else shares of the box `dims` (length, width, height); centred on the box's `center`."""
    physics = physics or {}
    base = physics.get("wheel_base") or KINEMATIC_BASE * dims[0]
    tread = physics.get("wheel_tread") or KINEMATIC_TREAD * dims[1]
    radii = [w.get("radius") for w in physics.get("wheels") or [] if w.get("radius")]
    radius = sum(radii) / len(radii) if radii else KINEMATIC_RADIUS
    cx, cy = center[0], center[1]
    return ([(cx + sx * base / 2, cy + sy * tread / 2) for sx in (1, -1) for sy in (1, -1)],
            float(radius), KINEMATIC_WIDTH)


class SurfaceLayer:
    def __init__(self, stage, surface_map, map_origin=(0.0, 0.0, 0.0), apply_force=None,
                 make_tiles=None):
        """`apply_force(body path, force, position)`, world frame, N and m: a force on a body
        for the next physics step (the soil's resistance); None: the soil sets friction only.
        `make_tiles(zone, spacing, origin)`: the ground of a zone with `ground_z`, an object with
        `update(grid)`, `clear()` and `remove()` (`soil_tiles.SoilTiles`); None: no ruts in
        the collision."""
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
        self.soil_xy = {}  # (vehicle path, wheel) -> its map-frame contact on soil
        self.tiles = {}  # zone id -> its own ground (make_tiles)
        # vehicle path -> [previous (x, y, yaw), this frame's, wheels, radius, width, load]
        self.kinematic = {}
        self.ruts = None  # the rut set the soil started from (`swb.ruts`), None: fresh
        self.driven = False  # a wheel has driven on the soil since then
        self.saved = True  # nothing driven since the last save
        if make_tiles is not None:
            for z in surface_map.zones:
                if z.ground_z is not None:
                    self.tiles[z.id] = make_tiles(z, SOIL_SPACING, self.origin)

    def set_condition(self, friction_scale=1.0, wetness="dry", properties=None):
        """A scenario's road condition (OpenSCENARIO `RoadCondition`, 047 S2), from the next
        step on: the whole road state, replacing the last. `swb.ruts` (D5) puts a saved rut
        set (or fresh ground) in the soil zones now, only while no wheel has driven on the soil
        since the last fresh start or set: ruts swapped under a moving car are not physical.
        SurfaceError lists every problem and changes nothing."""
        condition, smap = surfaces.road_condition(self.base, friction_scale, wetness,
                                                  properties)
        if condition.ruts is not None and condition.ruts != self.ruts:
            if self.driven:
                raise surfaces.SurfaceError(
                    f"{surfaces.RUTS_PROPERTY}: {condition.ruts}: the soil has been driven on "
                    f"since {'set ' + self.ruts if self.ruts else 'the start'}; a rut set goes "
                    f"in before (the scenario's Init)")
            self._put_ruts(condition.ruts)
        self.condition, self.map = condition, smap
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
                self.soil_xy.pop((vehicle.path, i), None)
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
        self.driven = True
        self.saved = False
        share = soil.edge_weight(zone.polygon, x, y, SOIL_BLEND * 2 * r)
        self.soil_out[(vehicle.path, i)] = (out, share)
        self.soil_xy[(vehicle.path, i)] = (x, y)
        if self.apply_force is not None and out.resistance > 0 and share > 0:
            f = -out.resistance * share * (1.0 if speed >= 0 else -1.0)
            self.apply_force(vehicle.path, (f * dx, f * dy, f * dz), contact.position)
        soil_mu = self.condition.friction_scale * out.mu(contact.load) / table[table.reference].mu
        return share * soil_mu + (1 - share) * surfaces.peak_mu(table, s, self.condition, speed)

    def set_kinematic(self, path, x, y, yaw, wheels, mass):
        """Once a frame, before its substeps: kinematic vehicle `path` (world frame, yaw from
        +x) moves to this pose over the frame; `wheels` from `kinematic_wheels`, `mass` kg."""
        k = self.kinematic.get(path)
        prev = k[1] if k is not None else (x, y, yaw)
        if math.hypot(x - prev[0], y - prev[1]) > KINEMATIC_JUMP:
            prev = (x, y, yaw)  # a teleport, not a drive
        offsets, radius, width = wheels
        self.kinematic[path] = [prev, (x, y, yaw), offsets, radius, width,
                                mass * G / len(offsets)]

    def roll_kinematic(self, fraction, dt, frame_dt):
        """After a substep of `dt`, the egos' updates done: every kinematic vehicle at
        `fraction` of its way through the frame of `frame_dt` rolls its wheels in the grid
        zones under them."""
        ox, oy, _ = self.origin
        for path in sorted(self.kinematic):
            (x0, y0, a0), (x1, y1, a1), offsets, radius, width, load = self.kinematic[path]
            da = math.remainder(a1 - a0, math.tau)
            x, y, yaw = x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction, a0 + da * fraction
            c, s = math.cos(yaw), math.sin(yaw)
            speed = ((x1 - x0) * c + (y1 - y0) * s) / frame_dt
            for i, (bx, by) in enumerate(offsets):
                wx, wy = x + bx * c - by * s + ox, y + bx * s + by * c + oy
                key = (path, i)
                sf = self.map.at(wx, wy)
                zone = next((z for z in self.map.zones if z.id == sf.soil_zone), None) \
                    if sf.soil == "grid" else None
                if zone is None:
                    self.soil_xy.pop(key, None)
                    continue
                grid = self.grids.get(zone.id)
                if grid is None:
                    grid = self.grids[zone.id] = soil.SoilGrid(self.map.table[sf.cls].soil,
                                                               SOIL_SPACING)
                    grid.begin_step()
                    log.info("soil zone %s (%s): grid at %g m", zone.id, sf.cls, SOIL_SPACING)
                grid.wheel(key, wx, wy, yaw if speed >= 0 else yaw + math.pi, load, radius,
                           width, abs(speed), abs(speed), dt)
                self.soil_xy[key] = (wx, wy)
                self.driven = True
                self.saved = False
            if fraction >= 1.0:  # the frame done: without a new pose it stands here
                self.kinematic[path][0] = (x1, y1, a1)

    def soil_under(self, vehicle):
        """Each wheel's (soil.WheelSoil, share) of the last step, None off soil."""
        return [self.soil_out.get((vehicle.path, i)) for i in range(len(vehicle.wheel_paths))]

    def fresh_soil(self):
        """Every soil zone back to untouched ground (no ruts)."""
        self.grids = {}
        self.soil_out = {}
        self.soil_xy = {}
        self.ruts, self.driven, self.saved = None, False, True
        for t in self.tiles.values():
            t.clear()

    def _put_ruts(self, name):
        """Rut set `name` (or fresh ground) in the soil zones, their tiles at once."""
        if name == surfaces.RUTS_FRESH:
            self.fresh_soil()
            log.info("ruts: fresh ground")
            return
        path = surfaces.ruts_dir(self.base) / f"{name}.npz"
        try:
            saved = soil.load_ruts(path)
        except soil.SoilError as e:
            raise surfaces.SurfaceError(f"{surfaces.RUTS_PROPERTY}: {e}") from e
        zones = {z.id: z for z in self.base.zones if z.soil == "grid"}
        problems = [f"{surfaces.RUTS_PROPERTY}: {path}: zone {zid!r} is not a soil zone on the "
                    f"grid in {self.base.path}" for zid in saved if zid not in zones]
        problems += [f"{surfaces.RUTS_PROPERTY}: {path}: zone {zid!r} was saved at "
                     f"{h:g} m, its grid is {SOIL_SPACING:g} m"
                     for zid, (h, _) in saved.items() if zid in zones and h != SOIL_SPACING]
        if problems:
            raise surfaces.SurfaceError("\n".join(problems))
        self.fresh_soil()
        for zid, (h, blocks) in saved.items():
            grid = self.grids[zid] = soil.SoilGrid(self.base.table[zones[zid].cls].soil, h)
            grid.load(blocks)
            if zid in self.tiles:
                self.tiles[zid].show(grid)
        self.ruts = name
        deepest = {zid: max((float(a.max()) for a in b.values()), default=0.0)
                   for zid, (_, b) in saved.items()}
        log.info("ruts: set %s from %s (%s)", name, path, ", ".join(
            f"{zid} {len(saved[zid][1])} block(s), {d * 100:.1f} cm deepest"
            for zid, d in deepest.items()))

    def save_ruts(self, directory):
        """The soil's ruts to `<directory>/<time>.npz` if a wheel drove on it since the last
        save or set: the path, or None."""
        if self.saved or not self.grids:
            return None
        import time as clock

        from pathlib import Path

        d = Path(directory)
        d.mkdir(parents=True, exist_ok=True)
        path = d / f"{clock.strftime('%Y-%m-%d_%H-%M-%S')}.npz"
        soil.save_ruts(path, self.grids)
        self.saved = True
        log.info("ruts saved: %s (started from %s); copy it to %s/<name>.npz for swb.ruts",
                 path, self.ruts or "fresh ground", surfaces.ruts_dir(self.base))
        return path

    def flush(self):
        """Once per frame, after the substeps: the grids' new ruts into their zones' ground
        (budgeted); {zone id: tiles rebuilt}."""
        out = {}
        wheels = list(self.soil_xy.values())
        for zid, tiles in self.tiles.items():
            grid = self.grids.get(zid)
            if grid is not None:
                done = tiles.update(grid, wheels)
                if done:
                    out[zid] = done
        return out

    def remove_tiles(self):
        """The zones' own ground off the stage (another world is loading)."""
        for t in self.tiles.values():
            t.remove()
        self.tiles = {}

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
        self.kinematic.pop(vehicle_path, None)
        self.soil_out = {k: v for k, v in self.soil_out.items() if k[0] != vehicle_path}
        self.soil_xy = {k: v for k, v in self.soil_xy.items() if k[0] != vehicle_path}


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
