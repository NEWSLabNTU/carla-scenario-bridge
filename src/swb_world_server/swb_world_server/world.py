"""`World`: what a simulator implements to be driven over the world protocol (roadmap 046).

Subclass it, set the class attributes, implement the core methods, and hand an instance to
`swb_world_server.server.WorldServer` (or run `swb_world_server.server.run`). The shell does
the protocol: versions, the owner lease, handles, validation, the clock, timing. A method
raises to refuse; its message is the error the bridge passes on to SSv2.

Frames are the protocol's: the world frame is right-handed and Z-up, metres and radians; the
bridge has taken the map origin off. Poses are ``((x, y, z), (qw, qx, qy, qz))`` at the
SSv2 entity origin, twists ``((vx, vy, vz), (wx, wy, wz))`` in the world frame at the entity
origin.
"""
from typing import NamedTuple, Optional


class Drive(NamedTuple):
    """What a step does with one body (`EntityPose`): a kinematic body goes to `pose`; a
    tracked one takes `twist`, after being put at `pose` when `teleport`; `kinematic` hands
    it to kinematic from this step on (then it goes to `pose`). An `EXTERNAL` body's drive is
    empty."""

    pose: Optional[tuple] = None
    twist: Optional[tuple] = None
    teleport: bool = False
    kinematic: bool = False


class World:
    """A simulator's world, driven by the bridge. The core is `load`, `spawn`, `despawn` and
    `step`; the rest goes with a capability."""

    #: The simulator, as the map directory's `<name>/world.yaml` names it.
    simulator_name = "unknown"
    #: What the server offers beyond the core: names from `protocol.CAPABILITY`
    #: (`SIGNALS`, `TRACKED_MOTION`, `CONTACTS`, ...).
    capabilities = ()
    #: The longest physics step the simulator takes, s; the bridge splits SSv2's step into
    #: substeps no longer than this. 0: it takes SSv2's step whole.
    max_physics_dt = 0.0

    @property
    def simulator_version(self):
        """For the log."""
        return ""

    def ready(self):
        """`(ready, why not)`: whether a world can be loaded now."""
        return True, ""

    def load(self, world, ground_paths, signals, request=None):
        """Replace the environment with `world` (the `world.yaml`'s string; empty: the
        default) and remove every entity. `signals` are the map's traffic signals (dicts:
        ``way_id``, ``regulatory_element_ids``, ``opendrive_id``, ``position`` or None), every
        one green until set; empty unless `SIGNALS`. `request` is the `LoadWorld` itself, for
        what else it carries (the map directory, the step time)."""
        raise NotImplementedError

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="", motion="KINEMATIC"):
        """One body: `kind` ``EGO``, ``VEHICLE``, ``PEDESTRIAN`` or ``MISC_OBJECT``; the box
        `center` (from the entity origin) and `dims`; `motion` ``KINEMATIC``, ``TRACKED``
        (`TRACKED_MOTION`) or ``EXTERNAL`` (the ego). May return a dict for the reply's
        `Body` (``model``, ``box_location``, ``box_extent``, ``origin_offset``); None: the
        declared box at the entity origin."""
        raise NotImplementedError

    def despawn(self, handle):
        raise NotImplementedError

    def step(self, drives, substeps, dt, render, sim_time):
        """Apply ``{handle: Drive}`` (``sim_time`` is the clock at the step's start), run
        `substeps` physics steps of `dt`, and return ``[(handle, pose, twist)]`` for every
        entity (``twist`` None where the simulator has none) and ``[(handle, other,
        other_path, impulse)]`` for the contacts (``other`` 0: the environment)."""
        raise NotImplementedError

    def set_signals(self, colours):
        """`SIGNALS`: ``{opendrive_id: colour}``, colour ``OFF``, ``RED``, ``AMBER`` or
        ``GREEN``, from the next step on."""
        raise NotImplementedError("this simulator has no traffic signals")

    def set_road_condition(self, friction_scale, wetness, properties):
        """`ROAD_CONDITION`: OpenSCENARIO's road condition, the whole road state from the next
        step on (`wetness` one of OpenSCENARIO's levels; `properties` the `swb.` ones,
        name to string). Raise ValueError, listing every problem, for what cannot be
        applied, and change nothing then. `load` and `reset` go back to the default."""
        raise NotImplementedError("this simulator has no road condition")

    def reset(self, handles):
        """A new owner, or the old one released the world: every entity goes. The world
        stays loaded."""
        for handle in handles:
            self.despawn(handle)

    def close(self):
        """The server is stopping."""
