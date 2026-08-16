# Two Autoware installs on one host

A host that has **both** the Autoware 1.5.0 Debian (`/opt/autoware/1.5.0`) and an
Autoware installed into the base ROS prefix (`/opt/ros/humble`, e.g. `autoware-full`
from `/opt/autoware-localrepo`) cannot build this workspace without help. This note is
what that looks like, why, and what to do.

## Symptom

`traffic_simulator` fails to compile against headers it does not use:

```
/opt/ros/humble/include/simulation_interface/conversions.hpp:53:10:
    fatal error: zmqpp/zmqpp.hpp: No such file or directory

src/.../lanelet_wrapper/lanelet_loader.cpp:53:79: error: no matching function for call to
    'lanelet::projection::format_v2::TransverseMercatorProjector::TransverseMercatorProjector(
        lanelet::Origin&, const double&)'

src/.../entity/ego_entity.cpp:255:42: error: 'RouteOption' is not a member of
    'concealer::FieldOperatorApplication'
```

Every one of these is our source being compiled against **someone else's header**.

## Why

`find_package` gets it right. Checked in `build/traffic_simulator/CMakeCache.txt`:

```
simulation_interface_DIR = <workspace>/install/simulation_interface/share/...
geometry_DIR             = <workspace>/install/geometry/share/...
concealer_DIR            = <workspace>/install/concealer/share/...
```

The **include search order** gets it wrong. Both prefixes contribute their include
directory as `-isystem`, and `-isystem` directories are searched in command-line order.
`/opt/ros/humble/include` lands at position 52 of `CXX_INCLUDES` while
`install/simulation_interface/include` lands at 72, so `#include
<simulation_interface/conversions.hpp>` finds the base-prefix copy first — which belongs
to a different, older release, and pulls in a `zmqpp` dev package we never asked for.

The obvious fixes do not work:

- **`CPATH`** is searched *after* every `-I` and `-isystem` option. It cannot win.
- **`-I<workspace>/install/<pkg>/include`** is silently dropped. GCC's rule: *"If a
  standard system include directory, or a directory specified with `-isystem`, is also
  specified with `-I`, the `-I` option is ignored."* CMake already passes that exact
  directory as `-isystem`, so the `-I` is discarded and the original order stands.

## The real fix

Remove the duplicate. Autoware 1.5.0 under `/opt/autoware/1.5.0` is self-contained (454
packages) and chains to `/opt/ros/humble` only for base ROS 2. The Autoware in the base
prefix is redundant here — and it ships its own scenario_simulator_v2 (16 packages),
which is what shadows this workspace's fork:

```bash
sudo apt remove --purge autoware-full autoware-config autoware-runtime
sudo apt autoremove --purge      # check the list before agreeing
```

Confirm `ros-humble-desktop` (or `ros-humble-ros-base`) is marked manually installed
first, or autoremove will take base ROS with it.

## The workaround, if you cannot remove it

Give the compiler a `-I` directory that is **not** also an `-isystem` directory. A flat
directory of symlinks to the real include trees satisfies that, and `-I` then beats every
`-isystem` entry:

```bash
# Workspace SSv2 headers
SHADOW=$HOME/.local/ssv2_include_shadow
rm -rf "$SHADOW"; mkdir -p "$SHADOW"
for d in install/*/include; do
    [ -d "$d" ] || continue
    for entry in "$d"/*; do ln -sfn "$PWD/$entry" "$SHADOW/$(basename "$entry")"; done
done

# Autoware 1.5.0 headers, so its API wins over the base prefix's older one
SHADOW2=$HOME/.local/autoware150_include_shadow
rm -rf "$SHADOW2"; mkdir -p "$SHADOW2"
for entry in /opt/autoware/1.5.0/include/*; do
    ln -sfn "$entry" "$SHADOW2/$(basename "$entry")"
done

colcon build --base-paths src --symlink-install \
    --cargo-args --profile dev-release \
    --cmake-args -DBUILD_TESTING=OFF \
    "-DCMAKE_CXX_FLAGS=-I$SHADOW -I$SHADOW2"
```

The workspace shadow has to be regenerated as packages install, so a from-scratch build
takes more than one pass: build, regenerate the shadow, build again. Order matters —
workspace headers first, then Autoware 1.5.0, then whatever the base prefix has left.

**Wipe the build directories between passes.** Anything compiled while the shadow was
still incomplete was compiled against the base prefix's headers, and colcon will not
rebuild it just because the flags changed underneath. The failure that follows looks
nothing like a header problem:

```
/usr/bin/ld: libsimple_sensor_simulator_component.so: undefined reference to
  `traffic_simulator::TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray_<...>
   >::generateMessage(...)'
```

`nm -DC install/traffic_simulator/lib/libtraffic_simulator.so` shows that symbol present
and exported — the stale object is simply looking for a differently-mangled one from the
other release. `rm -rf build/<ssv2 packages>` (keep `install/`, the shadow points into
it) and build again.

This is a workaround, not a fix. It papers over an environment that has two versions of
the same API visible at once; anything it does not cover will fail the same way.
