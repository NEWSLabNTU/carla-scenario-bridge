"""CARLA's walker ids -> blueprints, read from WalkerFactory's package (roadmap 044, open item 3).

    python3 walker_factory_ids.py $CARLA_SRC/carla/Unreal/CarlaUE4/Content/Carla/Blueprints/Walkers/WalkerFactory.uasset \
        > ../csb_isaac/walker_blueprints.json

WalkerFactory builds its pedestrians in its Blueprint graph, which an editor script cannot run
in a commandlet. The graph's compiled bytecode has them all: per pedestrian an EX_StringConst
with its id ("0001"), an EX_ObjectConst with its class (an import of the package) and two
EX_ByteConst, its gender and age. So this reads the package's name and import tables (UE 4.26)
and that bytecode, no editor needed.
"""
import json
import re
import struct
import sys

PACKAGE_TAG = 0x9E2A83C1
PKG_FILTER_EDITOR_ONLY = 0x80000000
# VER_UE4_NON_OUTER_PACKAGE_IMPORT: an import also carries its package's name (editor builds).
VER_IMPORT_PACKAGE_NAME = 520
VER_NAME_HASHES = 504
VER_LOCALIZATION_ID = 516
GENDERS = ("other", "female", "male")  # EPedestrianGender
AGES = ("child", "teenager", "adult", "elderly")  # EPedestrianAge
# EX_StringConst "NNNN", EX_ObjectConst <import>, EX_ByteConst <gender>, EX_ByteConst <age>
PEDESTRIAN = re.compile(rb"\x1f(\d{4})\x00\x20(.{4})\x24(.)\x24(.)", re.S)


class Package:
    def __init__(self, data):
        self.d, self.p = data, 0
        if self._u32() != PACKAGE_TAG:
            raise ValueError("not an Unreal package")
        if self._i32() != -4:
            self._i32()  # LegacyUE3Version
        self.version = self._i32()
        self._i32()  # licensee version
        custom_versions = self._i32()
        self.p += 20 * custom_versions  # FGuid + int32 each
        self._i32()  # TotalHeaderSize
        self._str()  # FolderName
        self.flags = self._u32()
        name_count, name_offset = self._i32(), self._i32()
        editor = not self.flags & PKG_FILTER_EDITOR_ONLY
        if self.version >= VER_LOCALIZATION_ID and editor:
            self._str()
        self._i32(), self._i32()  # gatherable text
        self._i32(), self._i32()  # exports
        import_count, import_offset = self._i32(), self._i32()
        self.p = name_offset
        self.names = []
        for _ in range(name_count):
            self.names.append(self._str())
            if self.version >= VER_NAME_HASHES:
                self.p += 4
        size = 36 if self.version >= VER_IMPORT_PACKAGE_NAME and editor else 28
        self.imports = [(struct.unpack_from("<i", data, import_offset + k * size + 16)[0],
                         self._name(import_offset + k * size + 20)) for k in range(import_count)]

    def _i32(self):
        v = struct.unpack_from("<i", self.d, self.p)[0]
        self.p += 4
        return v

    def _u32(self):
        v = struct.unpack_from("<I", self.d, self.p)[0]
        self.p += 4
        return v

    def _str(self):
        n = self._i32()
        if n < 0:
            s = self.d[self.p:self.p - 2 * n].decode("utf-16-le")
            self.p -= 2 * n
        else:
            s = self.d[self.p:self.p + n].decode("latin-1")
            self.p += n
        return s.rstrip("\0")

    def _name(self, offset):
        index, number = struct.unpack_from("<ii", self.d, offset)
        return self.names[index] + (f"_{number - 1}" if number else "")

    def import_path(self, index):
        """An import's full path (`index` is a negative package index)."""
        outer, name = self.imports[-index - 1]
        return f"{self.import_path(outer)}.{name}" if outer < 0 else name


def walker_table(data):
    pkg = Package(data)
    table = {}
    for m in PEDESTRIAN.finditer(data):
        index = struct.unpack("<i", m.group(2))[0]
        if index >= 0:
            continue
        cls = pkg.import_path(index)  # /Game/.../BP_Walker_X.BP_Walker_X_C
        blueprint = cls.rsplit(".", 1)[-1].removesuffix("_C")
        table[f"walker.pedestrian.{m.group(1).decode()}"] = {
            "blueprint": blueprint, "class": cls,
            "gender": GENDERS[m.group(3)[0]] if m.group(3)[0] < len(GENDERS) else "other",
            "age": AGES[m.group(4)[0]] if m.group(4)[0] < len(AGES) else "adult"}
    return dict(sorted(table.items()))


if __name__ == "__main__":
    with open(sys.argv[1], "rb") as f:
        table = walker_table(f.read())
    if not table:
        sys.exit("no pedestrians found: not WalkerFactory, or another engine version")
    json.dump(table, sys.stdout, indent=1)
    print()
