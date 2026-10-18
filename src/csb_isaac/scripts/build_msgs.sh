#!/usr/bin/env bash
# Autoware's ROS message packages for Isaac's Python 3.12 (roadmap 038).
#
# The world server publishes Autoware's vehicle status and takes its control commands with
# Isaac's bundled rclpy, which is built for Python 3.12; Autoware's message bindings are
# built for the system's 3.10. The Humble install ships the C sources rosidl generated for
# each package's Python bindings, so this compiles those against 3.12 and numpy, links
# them with the install's (Python-independent) C type support, and puts both in
# $ISAAC_ROOT/msgs (`isaac_env.sh` adds it to PYTHONPATH and LD_LIBRARY_PATH).
#
#     src/csb_isaac/scripts/build_msgs.sh [package...]
set -euo pipefail
I=${ISAAC_ROOT:-/mnt/disk1/isaac-sim}
PREFIX=${AUTOWARE_PREFIX:-/opt/autoware/1.5.0}
OUT=$I/msgs
PKGS=("$@")
[ ${#PKGS[@]} -gt 0 ] || PKGS=(autoware_vehicle_msgs autoware_control_msgs)
PYINC=$("$I/venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["include"])')
NPINC=$("$I/venv/bin/python" -c 'import numpy; print(numpy.get_include())')
ISL=$I/venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/humble/lib
mkdir -p "$OUT/lib"
inc=(-I"$PYINC" -I"$NPINC")
for d in /opt/ros/humble/include/*_msgs /opt/ros/humble/include/*_interfaces \
         /opt/ros/humble/include/rosidl_* /opt/ros/humble/include/rcutils "$PREFIX"/include/*_msgs; do
    inc+=(-I"$d")
done
# NDEBUG: the generated code asserts PyUnicode_IS_READY, which 3.12 no longer guarantees.
# DT_RPATH (not RUNPATH): $ORIGIN must win over LD_LIBRARY_PATH, or a 3.10 build of the same
# library elsewhere on the path is loaded instead.
cflags=(-DNDEBUG -O2 -fPIC -shared -w)
rpath=(-Wl,--disable-new-dtags,-rpath,'$ORIGIN:$ORIGIN/../lib')
for pkg in "${PKGS[@]}"; do
    src=$PREFIX/local/lib/python3.10/dist-packages/$pkg
    [ -d "$src" ] || { echo "no $src" >&2; exit 1; }
    rm -rf "${OUT:?}/$pkg"
    mkdir -p "$OUT/$pkg"
    (cd "$src" && find . -name '*.py' -exec cp --parents {} "$OUT/$pkg/" \;)
    cp "$PREFIX"/lib/lib"${pkg}"__rosidl_{generator_c,typesupport_c,typesupport_fastrtps_c,typesupport_introspection_c}.so "$OUT/lib/"
    # Message dependencies' Python support (std_msgs, builtin_interfaces...) is Isaac's.
    deps=()
    for d in $(ldd "$src/lib${pkg}__rosidl_generator_py.so" | sed -n 's/.*lib\([a-z0-9_]*\)__rosidl_generator_py\.so.*/\1/p'); do
        [ "$d" = "$pkg" ] || deps+=(-l"${d}__rosidl_generator_py")
    done
    gcc "${cflags[@]}" "${inc[@]}" "$src"/*/_*_s.c -o "$OUT/$pkg/lib${pkg}__rosidl_generator_py.so" \
        -L"$OUT/lib" -l"${pkg}__rosidl_generator_c" -L"$ISL" "${deps[@]}" -lrosidl_runtime_c -lrcutils "${rpath[@]}"
    for ts in c fastrtps_c introspection_c; do
        gcc "${cflags[@]}" "${inc[@]}" "$src/_${pkg}_s.ep.rosidl_typesupport_${ts}.c" \
            -o "$OUT/$pkg/${pkg}_s__rosidl_typesupport_${ts}.cpython-312-x86_64-linux-gnu.so" \
            -L"$OUT/$pkg" -l"${pkg}__rosidl_generator_py" -L"$OUT/lib" -l"${pkg}__rosidl_typesupport_${ts}" \
            -l"${pkg}__rosidl_generator_c" "${rpath[@]}"
    done
    echo "built $pkg into $OUT"
done
