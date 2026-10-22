"""The world protocol (`proto/csb_world.proto`, roadmaps 037, 046), Python side.

The message classes are built at import from `csb_world.desc`, the descriptor set protoc
writes for the .proto (`scripts/gen_world_proto.sh`), not from generated `_pb2` code: the
host's protoc (3.12) writes `_pb2` files that Isaac's protobuf runtime (7.x) refuses, while
a descriptor set reads the same in every runtime. `test_server` checks the file is current.
"""
from pathlib import Path

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

# Bump with the bridge's `backend::remote::PROTOCOL_VERSION` and `csb_core::world_server`'s
# on any change a peer of the other version would misread.
PROTOCOL_VERSION = 4

DESCRIPTOR_SET = Path(__file__).with_name("csb_world.desc")


def _load():
    files = descriptor_pb2.FileDescriptorSet.FromString(DESCRIPTOR_SET.read_bytes())
    pool = descriptor_pool.DescriptorPool()
    for f in files.file:
        pool.Add(f)
    names = [m.name for f in files.file for m in f.message_type]
    classes = {n: message_factory.GetMessageClass(pool.FindMessageTypeByName(f"csb_world.{n}"))
               for n in names}
    enums = {e.name: {v.name: v.number
                      for v in pool.FindEnumTypeByName(f"csb_world.{e.name}").values}
             for f in files.file for e in f.enum_type}
    return classes, enums


_CLASSES, _ENUMS = _load()
globals().update(_CLASSES)
KIND = _ENUMS["Kind"]
MOTION = _ENUMS["Motion"]
COLOUR = _ENUMS["Colour"]
# Capabilities by their short name (`SIGNALS`), as `World.capabilities` lists them.
CAPABILITY = {k.removeprefix("CAPABILITY_"): v for k, v in _ENUMS["Capability"].items()
              if k != "CAPABILITY_UNSPECIFIED"}
KIND_NAME = {v: k for k, v in KIND.items()}
MOTION_NAME = {v: k for k, v in MOTION.items()}
COLOUR_NAME = {v: k for k, v in COLOUR.items()}
CAPABILITY_NAME = {v: k for k, v in CAPABILITY.items()}


def pose_tuple(pose):
    """`((x, y, z), (qw, qx, qy, qz))` of a `Pose`; an unset orientation is the identity."""
    p, q = pose.position, pose.orientation
    if not pose.HasField("orientation"):
        return (p.x, p.y, p.z), (1.0, 0.0, 0.0, 0.0)
    return (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)


def make_pose(position, quat_wxyz):
    """A `Pose` from `(x, y, z)` and `(qw, qx, qy, qz)`."""
    w, x, y, z = quat_wxyz
    return Pose(position=Vec3(x=position[0], y=position[1], z=position[2]),  # noqa: F821
                orientation=Quat(x=x, y=y, z=z, w=w))  # noqa: F821


def twist_tuple(twist):
    """`((vx, vy, vz), (wx, wy, wz))` of a `Twist`."""
    v, w = twist.linear, twist.angular
    return (v.x, v.y, v.z), (w.x, w.y, w.z)


def make_twist(linear, angular):
    """A `Twist` from `(vx, vy, vz)` and `(wx, wy, wz)`."""
    return Twist(linear=Vec3(x=linear[0], y=linear[1], z=linear[2]),  # noqa: F821
                 angular=Vec3(x=angular[0], y=angular[1], z=angular[2]))  # noqa: F821
