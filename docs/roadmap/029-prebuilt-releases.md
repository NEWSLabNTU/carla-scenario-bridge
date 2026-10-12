# Phase 029: Prebuilt Releases

A user installs the simulation side and the vehicle side from release tarballs instead of
building them: no Rust toolchain, no colcon, no SSv2 build. Today setup means the README's
`colcon build` of 62 packages -- the SSv2 fork (a large C++ build), carla-rust (which
compiles LibCarla), the Rust bridge, acb -- after installing rustup and colcon's cargo
plugin.

**Source**: user direction 2026-10-13 ("prebuilt binaries for users other than docker";
"tarball of the built workspace is preferred"; release strategy across ROS distros and
CARLA versions agreed the same day).
**Depends on**: [018](018-install-and-world-config.md) (the user install, the CARLA
version as a Cargo feature), [027](027-user-documentation.md) (README setup, user guide),
[030](030-agent-api.md) (the agent SDK and the Autoware adapter the vehicle side ships).

## Decisions (user, 2026-10-13)

- **Tarballs of the built workspace** are the release format. Not Docker; `.deb` packages,
  conda / pixi and snap / AppImage were considered and left out (below).
- **Python only** for anything new a user runs.
- **Few builds, not one per combination.** The release is split by side, every supported
  CARLA version ships in the same tarball, and the simulation side does not depend on an
  Autoware install. Several ROS distros and CARLA versions are expected over time.

## What a build depends on (measured 2026-10-13, this host)

- `carla_scenario_bridge` links LibCarla statically: no CARLA library at run time. Outside
  ROS it needs `libspdlog` and `libfmt` (apt) and the C/C++ runtime. Its RPATH carries
  `$ORIGIN`-relative entries next to the absolute build path, and the colcon setup scripts
  hold no absolute path: the tree is relocatable as built (the absolute entries are
  stripped from a release).
- The binary is 355 MB as built (`dev-release`, debug info); the release profile and
  `strip` cut that to tens of MB (measured in step 2).
- **The CARLA version touches two executables only**: `carla_scenario_bridge` and
  `acb_bridge`. carla-rust's API differs per CARLA release and two version features in one
  dependency graph are a build error, so each CARLA version is a separate binary -- but
  nothing else in the tree changes with it.
- **The simulation side needs 13 Autoware packages, none of them an Autoware install.**
  The bridge, `csb_launch`, `csb_interfaces` and `csb_examples` depend on nothing from
  Autoware. The relay and the SSv2 fork depend on message packages
  (`autoware_adapi_v1_msgs`, `autoware_vehicle_msgs`, `autoware_system_msgs`,
  `autoware_control_msgs`, `autoware_perception_msgs`, `autoware_map_msgs`,
  `autoware_internal_planning_msgs`, `tier4_rtc_msgs`, `tier4_external_api_msgs`,
  `tier4_simulation_msgs`, `tier4_planning_msgs`, `tier4_debug_msgs`) and one library
  (`autoware_lanelet2_extension`), all buildable from public source. Since 017 the
  simulation side reaches the ego only over the relay's TCP connection, never over ROS, so
  those packages never meet a running Autoware.
- **The vehicle side does talk to Autoware over ROS** (acb's vehicle interface, the agent's
  AD API calls), so it must match the Autoware release, and through it the ROS distro.

## Design

### Artefacts

| Artefact | Built per | Contents |
|---|---|---|
| **Simulation side** `csb-sim-<ver>-<distro>-<arch>.tar.zst` | ROS distro x arch | SSv2 fork, the bridge for every supported CARLA version, relay, `csb_launch`, `csb_examples`, `csb_interfaces`, the 13 Autoware packages above |
| **Vehicle side for Autoware** `csb-vehicle-<ver>-autoware<release>-<arch>.tar.zst` | Autoware release (which fixes the distro) x arch | acb's packages with `acb_bridge` for every supported CARLA version, the Autoware adapter (030) |
| **Agent SDK** `csb_agent-<ver>-py3-none-any.whl` | nothing | 030's pure-Python SDK, for any autopilot; also inside both tarballs |

A user with Autoware takes the simulation side and the vehicle side; a user with another
autopilot takes the simulation side and the SDK, with no Autoware anywhere. For two distros
and two Autoware releases that is four compiled builds and one wheel per release, instead of
one build per distro x Autoware x CARLA combination.

Each tarball is a plain `colcon build --merge-install` (not `--symlink-install`), Release /
`--release`, binaries stripped, `build/` and `log/` left out, plus: `setup.bash` (colcon's,
with the guard below), `csb-release.yaml` (side, distro, Autoware release for the vehicle
side, architecture, the git SHAs of this repo and every submodule and vendored package,
and each CARLA binary with its support tier), `install-deps.sh`, `LICENSES/`.

### Every CARLA version in one tarball

The two CARLA-dependent executables are built once per supported CARLA version and
installed side by side, with a launcher under the usual name:

```
lib/carla_scenario_bridge/carla_scenario_bridge-0.9.16
lib/carla_scenario_bridge/carla_scenario_bridge-0.9.15
lib/carla_scenario_bridge/carla_scenario_bridge-0.9.14
lib/carla_scenario_bridge/carla_scenario_bridge          # launcher
```

(and the same for `acb_bridge`). The launcher picks the binary from `carla_version:=` (a
launch argument and an environment variable) when given, and otherwise from the server: it
reads the version the CARLA server reports and starts the matching binary, or stops naming
the server's version and the versions it has. How it reads the server's version without a
CARLA client of its own -- CARLA's RPC `version` call from the launcher, or one binary's
existing version check exiting with the server's version for the launcher to act on -- is
settled in step 2. A source build keeps today's single binary; the launcher is a release
artefact.

### Vendored Autoware packages (simulation side)

A `release/sim.repos` file pins the 13 packages' repositories at the versions of the
Autoware release the fork is tested with (1.5.0 today); the simulation-side build imports
them (`vcs import`) into the build tree. They are built and shipped inside the simulation
tarball only, so the simulation side needs no `/opt/autoware`. Because they never exchange
messages with a running Autoware (the relay's link is TCP), a different Autoware release on
the vehicle host cannot meet them. A source build of this workspace keeps building against
the user's Autoware as today.

### Install, for a user

```bash
tar -xf csb-sim-<...>.tar.zst -C ~/csb-sim
~/csb-sim/install-deps.sh                      # prints (or with --apply runs) one apt line
source /opt/ros/humble/setup.bash
source ~/csb-sim/setup.bash                    # refuses a distro / arch mismatch
ros2 run csb_launch csb_setup                  # optional: download CARLA (and maps, below)

# vehicle host (may be the same machine), for Autoware:
tar -xf csb-vehicle-<...>.tar.zst -C ~/csb-vehicle
source /opt/autoware/1.5.0/setup.bash
source ~/csb-vehicle/setup.bash                # refuses an Autoware / distro / arch mismatch
```

- `install-deps.sh` holds the apt packages rosdep would install for the tarball's packages
  that the base it sits on (ROS for the simulation side, ROS + Autoware for the vehicle
  side) does not provide, generated at build time from `rosdep keys` / `rosdep resolve`.
- The `setup.bash` guard reads `csb-release.yaml` and compares `ROS_DISTRO`, the
  architecture and, for the vehicle side, the sourced Autoware's release, and stops with
  both values named on a mismatch.
- `csb_setup` downloads a CARLA release the tarball has a binary for (0.9.16 by default)
  into a directory the user names, checks its version and prints the `CarlaUE4.sh` line.
  Maps: when the map pack is published (a separate decision, not taken here), `csb_setup
  --maps` fetches it; until then it prints the user guide's map section.

### Support tiers

| Tier | Meaning | Today |
|---|---|---|
| **Tested** | built, and the starter suites run against a real CARLA before the release | Humble, Autoware 1.5.0, CARLA 0.9.16 |
| **Built** | compiled and smoke-tested in CI, never run against a real CARLA | CARLA 0.9.14, 0.9.15 |
| **Source only** | documented, no artefact | CARLA 0.10 (until 019's port builds the vehicle side) |

ROS distros follow the LTS releases (Humble to 2027-05, Jazzy to 2029-05); Autoware the
latest two releases. Each release's notes carry this table; `csb-release.yaml` carries
each CARLA binary's tier. **Jazzy is a phase of its own, not part of 029**: the SSv2 fork
and acb are Humble-only today, Jazzy needs Ubuntu 24.04, and its DDS type hashes make
mismatched message versions fail to connect rather than misbehave. 029 builds the pipeline
so a distro is one more matrix entry once that port exists.

### Build, in CI

A GitHub Actions workflow on a tag (`v*`):

1. **Simulation side**, per distro (Humble on Ubuntu 22.04 today): ROS from apt;
   submodules; `vcs import` of `release/sim.repos`; `rosdep install`; Rust stable; one
   `colcon build --merge-install` of everything but the CARLA-dependent crates, then
   `carla_scenario_bridge` once per CARLA version (`CARLA_VERSION`), each installed under
   its versioned name; the launcher.
2. **Vehicle side**, per Autoware release: ROS and the Autoware release's packages the
   workspace builds against (from its binary release, the README's prerequisite); acb's
   packages and the Autoware adapter; `acb_bridge` once per CARLA version.
3. **Wheel**: `csb_agent`, built once.
4. **Caches**: `ccache` and the cargo cache keyed on the lockfiles and submodule SHAs (SSv2
   dominates the simulation-side time, LibCarla the per-CARLA-version time).
5. **Smoke test** per tarball on a clean job: extract to a fresh path, `install-deps.sh
   --apply`, source, `ros2 launch csb_launch simulation.launch.xml --show-args` (vehicle
   side: its launch's `--show-args`), each CARLA binary's `--version`, the launcher's
   selection with `carla_version:=`, the relay's and the SDK's unit tests.
6. **Upload** the tarballs, the wheel and their `sha256` to the GitHub release, with the
   tier table in the notes.

If GitHub's runners cannot build it (time, disk, or Autoware's packages), the same
workflow runs on a self-hosted runner on this host; the choice is recorded in the results.
The tested tier's live run (the starter suites) is done on this host before a release is
published, and its summary goes in the release notes.

### Considered, left out

`.deb` packages with an apt repository (62 packages to package; the Rust package needs
custom Debian rules; the SSv2 fork's package names can clash with SSv2 packages Autoware
installs) -- later, if users ask. Conda / pixi (no Autoware in conda, and a conda ROS beside
a system Autoware breaks). Snap / AppImage (DDS discovery inside confinement). Docker (not
wanted). One build per distro x Autoware x CARLA combination (the matrix above makes it
unnecessary). One binary for several CARLA versions (carla-rust's per-version APIs are
exclusive features).

## Steps

1. [x] **Docs.** This phase doc.
2. [ ] **Local release build.** `just release-sim` and `just release-vehicle` build the
   merge-install trees with every CARLA binary under its versioned name, strip them, write
   `csb-release.yaml` and the tarballs; measure sizes; settle the launcher's version source
   (spike); extract both on this host into fresh paths, source them, and run
   `town01_ego_drive` from the tarballs.
3. [ ] **Vendored packages.** `release/sim.repos`; a simulation-side build with no
   `/opt/autoware` on the path (a clean shell or container used only to prove it) that
   passes the relay's and the bridge's tests.
4. [ ] **Launcher.** Selection by argument, environment and server version; refusal naming
   both versions; tests for each path.
5. [ ] **Guard and dependencies.** The `setup.bash` guard, `install-deps.sh` generation,
   their tests (mismatched distro / Autoware release / arch refused).
6. [ ] **`csb_setup`.** CARLA download and version check; the map step as described.
7. [ ] **CI.** The workflow (simulation side per distro, vehicle side per Autoware release,
   the wheel), caches, the smoke-test jobs, release upload; one dry-run release on a
   pre-release tag.
8. [ ] **Docs for users.** README setup: "from a release" first, "from source" second, with
   the support tiers; user guide install section; the developer guide gets "cutting a
   release" (including the tested tier's live run).

## Acceptance

- On a host with ROS Humble and Autoware 1.5.0 and no Rust or colcon build: extract both
  tarballs, `install-deps.sh`, source, and the starter `basic/` suite passes under
  `run_suite` with both sides from the release, the CARLA binary picked by the launcher.
- The simulation side starts and serves a scenario with no Autoware install on its path
  (driven by 030's CARLA reference agent).
- A tag produces the tarballs, the wheel and checksums on the GitHub release without manual
  steps, and the notes carry the support tiers.
- The guard refuses a wrong distro, Autoware release or architecture with both values
  named; the launcher refuses a CARLA server it has no binary for, naming both versions.

### Results
