# Phase 018: Install with ROS Tools, and Configure the CARLA World over ROS

Finish 017's "ROS commands only" promise for the steps around a run: installing (`rosdep
install` + `colcon build`, no justfile), preparing Autoware's data directory, and configuring
the CARLA world (weather first) -- each a ROS command a user can type without this repo's
scripts.

**Source**: UX follow-up, user direction 2026-10-09: "go rosdep install and colcon build
procedure for users"; "we need a way to specify the Carla version as carla-rust supports many
CARLA client versions"; "go submodule instead of vcs"; "have ROS friendly way to set the
weather ... there will be several new commands to configure the Carla world"; the generated
`.cargo/config.toml` belongs to colcon-cargo-ros2, not to us.
**Depends on**: [017](017-user-workflow.md) (ROS-native run workflow, csb as a launched node).
**Relates to**: [005](005-hardening-awf-contribution.md) (setup guide).

## Problem

What `docs/user-guide.md` still asks of a user that is not a ROS command:

| Step | Today | Why it matters |
|---|---|---|
| Install | `just install-deps && just build` | the justfile is the developer's tool; a user's workspace has no `just` |
| CARLA version | `CARLA_VERSION=0.9.16`, set only by the justfiles | carla-rust (`carla/build.rs:45`) selects its API from that env var and **silently builds the 0.10 API when it is unset**: a plain `colcon build` compiles against the wrong CARLA, or fails with a convincing "no field `position`" |
| `color_names` | manual `git clone` into `src/` (ignored by git) | SSv2 needs it; nothing tells a fresh clone so |
| Autoware data | acb's `scripts/link_autoware_data.sh` | without a writable `data_path` the TensorRT nodes silently never appear (the script's header) |
| Weather | `scripts/set_weather.py ClearNoon` | needs the CARLA Python wheel on `PYTHONPATH`; and every map load resets the weather, so it does not survive a scenario on another town |

Not options:

- **`.cargo/config.toml` `[env]`** for the version: that file is generated per workspace by
  colcon-cargo-ros2 (between `BEGIN/END` markers) and is git-ignored; nothing we write there
  reaches a user.
- **SSv2 `EnvironmentAction`** for weather: the interpreter parses it and does nothing
  (`environment_action.cpp`: `run()` is `LINE()`), and the protocol has no weather message.
  Forwarding it is a fork change of its own; out of scope here (see Later).

## Design

### CARLA version is a Cargo feature (carla-rust)

carla-rust gains mutually exclusive features `carla-0915`, `carla-0916`, `carla-0100`; the
consumer names its CARLA in `Cargo.toml`:

```toml
carla = { git = "...", rev = "...", features = ["carla-0916"] }
```

- `build.rs` (carla-sys and carla) derives the version from the feature; two features are a
  build error naming both.
- `CARLA_VERSION` still wins when set (developer override, existing scripts).
- Neither set: **build error**, not a silent 0.10 -- the "never bare cargo" trap in CLAUDE.md
  disappears with it.
- Pushed to carla-rust main (fast-forward, allowed); csb's and acb's pins move to it.

Features are additive by Cargo's design; these are not. The build error covers the only way
that bites (two consumers in one graph picking different versions), and only our two
workspaces depend on carla-rust.

### Runtime version check

csb and acb compare the version they were built for (`carla::VERSION` / carla-sys's
`CARLA_VERSION`) with `client.server_version()` on every connect. A different release (major.minor.patch, suffixes such as `-dirty` ignored):
refuse with both versions in the message. A client built for 0.9.16 against a 0.9.15 server
today fails somewhere in RPC with a msgpack error.

### Install with rosdep and colcon

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/carla-scenario-bridge.git
cd carla-scenario-bridge
# once per machine: Rust (rustup) and the Rust colcon plugin -- the two things rosdep can't
curl https://sh.rustup.rs -sSf | sh && pip install colcon-cargo-ros2
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- `color_names` becomes a submodule at `src/color_names` (upstream OUXT-Polaris, pinned).
- Every system library the build needs is a rosdep key in some `package.xml`
  (`libclang-dev`, `protobuf-compiler`/`protobuf-dev`, `libzmq3-dev`, `python3-xmlschema`
  already via SSv2); `rosdep check --from-paths src --ignore-src` on a clean machine is
  the test.
- Plain `colcon build` (no `--symlink-install`) is the user path, which also sidesteps the
  setuptools `--editable` trap. `just build` stays the developer shortcut.

### Autoware data directory: `ros2 run acb_launch setup_autoware_data`

The script moves into `acb_launch` as an installed executable (same behaviour: mirror
`/opt/autoware/<ver>/data` into `~/autoware_data`, real dirs, symlinked files, built engines
kept). `carla_simulator.launch.xml` checks at launch that `data_path` exists and is writable
and refuses with the command to run, instead of nodes going missing.

### World configuration: services on the bridge

The bridge is the long-lived owner of the CARLA connection on the simulation side, so world
settings live there:

- New package `csb_interfaces` (rosidl): `SetWeather.srv` (a preset name, or explicit
  parameters: cloudiness, precipitation, precipitation_deposits, wind_intensity, fog_density,
  fog_distance, wetness, sun_azimuth_angle, sun_altitude_angle), `GetWeather.srv`.
- csb gains an rclrs node (as acb has) beside its ZMQ loop, serving
  `/carla/set_weather` and `/carla/get_weather` in the simulation domain. Requests are
  applied by the CARLA-owning thread between ticks.
- Parameter `weather` (preset name; empty = leave CARLA's), launch arg
  `simulation.launch.xml weather:=ClearNoon`. The bridge applies the configured weather on
  connect, and **re-applies the current weather after every map load**, so a weather choice
  survives a scenario on another town.
- Users type ROS commands:
  ```bash
  ros2 service call /carla/set_weather csb_interfaces/srv/SetWeather "{preset: ClearNoon}"
  ros2 service call /carla/get_weather csb_interfaces/srv/GetWeather
  ```
- Later world commands (rendering on/off, map load outside a scenario, ...) follow the same
  pattern: one srv in `csb_interfaces`, one handler on the bridge.

Rejected: a separate Python node with the CARLA wheel (a second client, a version-matched
wheel the user must install, and no hook into map loads).

## Steps

1. - [x] carla-rust: version features, precedence (`CARLA_VERSION` > feature), error when
   neither or two; unit check that each feature selects its cfg; push to main; csb + acb pins
   and `features = ["carla-0916"]`; `cargo check` with no env succeeds in both workspaces.
   *carla-rust `e06de1c` on master (features `carla-0914/0915/0916/0100`, also exported as
   `carla::CARLA_VERSION`). With `CARLA_VERSION` unset: `docs-only,carla-0915|0916|0100` set
   `carla_version_0915|0916|0100`; `carla-0915,carla-0916` fails "conflicting CARLA version
   features enabled: carla-0915, carla-0916"; no feature fails "no CARLA version selected";
   `CARLA_VERSION=0.9.15` + `carla-0916` gives 0915; a full `carla-0916` check downloads the
   0.9.16 prebuilt. `cargo check` with no env: csb and acb (acb_bridge, carla_pcd_gen,
   carla_manual_control) both compile, carla build output `carla_version_0916`.*
2. - [x] Runtime version check in csb and acb; message tested against a faked mismatch.
   *`carla_version.rs` in both (`check(built_for, server)`; release = major.minor.patch, so
   0.9.16 vs 0.9.15 is refused; build suffixes like `-dirty` ignored; an unparsable server
   version is refused). Unit tests: 0.9.16 vs 0.9.16-dirty ok, vs 0.9.15 / 0.10.0 refused
   with both versions in the message. csb: refused like a failed connect (startup loop and
   `reconnect_carla`); acb: `connect_to_carla` returns the error and the bridge exits.
   `just test` 187/187; acb nextest 10/10 (5 version tests). The rebuilt simulation side
   logged `CARLA server version 0.9.16 matches this build (0.9.16)` on connect (2026-10-10).*
3. - [x] `color_names` as a submodule; rosdep keys complete (`rosdep check` clean); user
   install section rewritten to rosdep + colcon; justfile `install-deps` reduced to the
   developer extras (nightly, nextest).
   *`src/color_names` submodule at `c6a3227` (OUXT-Polaris master). csb `package.xml` gains
   `protobuf-dev` (protoc + libprotobuf) and `libzmq3-dev`; acb drops the unresolvable keys
   `rclrs`, `rosidl_runtime_rs` (crates) and `cargo-ament-build` (obsolete tool).
   `rosdep check --from-paths src --ignore-src -r`: every key resolves; on this host only
   SSv2's test/lint deps are uninstalled (python3-xmlschema, yamale, replay-testing,
   ament-cmake-clang-format, ouxt-lint-common), and `rosdep install --simulate` exits 0.*
4. - [x] `setup_autoware_data` in acb_launch; launch-time writable check; script retired.
   *acb `ccfc64c`: `scripts/link_autoware_data.sh` → `acb_launch/scripts/setup_autoware_data`
   (`install(PROGRAMS)`, header kept, `--help`); `ros2 run acb_launch setup_autoware_data
   /opt/autoware/1.5.0/data <tmp>` linked 171 files. `carla_simulator.launch.xml` includes
   `check_data_path.launch.py` (an `OpaqueFunction` that raises) right after declaring
   `data_path`: `play_launch dump`/`launch` and `ros2 launch` with `data_path:=/nonexistent`
   or the root-owned `/opt/autoware/1.5.0/data` exit 1 with "data_path ... does not exist /
   is not writable ... Populate it with: ros2 run acb_launch setup_autoware_data"; a
   writable path resolves silently (148 nodes); `ros2 launch --show-args` is unaffected. The
   acb demo launch and justfile defaulted to a read-only / missing data path and now default
   to `~/autoware_data` too.*
5. - [x] `csb_interfaces` + rclrs node in csb; set/get weather services; `weather` param and
   launch arg; re-apply after map load; `scripts/set_weather.py` retired.
   *`src/csb_interfaces` (`Weather.msg`, `SetWeather.srv`: `preset` or `use_parameters` +
   `weather`; `GetWeather.srv`; both answer with the resulting values and matching preset).
   csb `world_services.rs`: rclrs 0.7 node `carla_scenario_bridge` on its own thread, async
   services that hand a `WorldCommand` to the ZMQ thread (which owns CARLA) and reply once it
   is applied between SSv2 requests. `weather.rs`: CARLA's 22 presets under their Python names
   (values from carla-rust `rpc::weather`), case/`_`-insensitive. Param `weather` (config file
   key, ROS param, `bridge.launch.xml`/`simulation.launch.xml weather:=`, `WEATHER=` for
   `just run`), validated at start. Applied on connect; re-applied after every `load_world`
   and every reconnect. `just test` 196/196 (9 new), `just check` clean. Live, 2026-10-10:
   set `{preset: ClearNoon}` → `success=True ... preset='ClearNoon'` (from CARLA's default
   cloudiness 60 / precipitation 40); `town02_episode_change` passed, the bridge logged
   `Map 'Town02' loaded` then `Weather ClearNoon applied (map load)`, and get_weather after
   it returned `preset='ClearNoon'`. `{}` and `{preset: Sunny}` are refused with the preset
   list.*
6. - [x] Docs: user guide, CLAUDE.md ("never bare cargo" rewritten), acb README.
   *User guide: `setup_autoware_data` in the fresh-machine notes, `weather:=` on the
   simulation side, new section 5 "Configure the CARLA world" (both services, presets,
   persistence across map loads). acb README: "Autoware data directory (once per machine)".
   CLAUDE.md: "The CARLA version is a Cargo feature" (db094cf) plus the world-config design
   note and the new files; `design/user-workflow.md` lists the `weather` param and services.*

## Acceptance

Met 2026-10-10.

- [x] A fresh clone, on a machine with ROS, Autoware and Rust, builds with exactly the commands
  in the user guide; no `just`, no env var.
  *`git clone --recurse-submodules` of `14132d1` into a scratch dir (submodules acb
  `ccfc64c`, color_names `c6a3227`, SSv2 `efe7f87`); `rosdep install --from-paths src
  --ignore-src -y --simulate` exits 0 (only SSv2's test/lint deps listed, as in step 3);
  `colcon build --packages-up-to carla_scenario_bridge csb_interfaces --cmake-args
  -DCMAKE_BUILD_TYPE=Release` with `CARLA_VERSION` unset: 2 packages, 3 min 8 s, its own
  target dir. The whole workspace (50 packages) builds the same way in the main checkout.
  Note: `-DCMAKE_BUILD_TYPE=Release` does not reach cargo -- colcon-cargo-ros2 built the
  Rust crates in the `dev` profile.*
- [x] `colcon build` with `CARLA_VERSION` unset compiles the 0.9.16 API; with
  `CARLA_VERSION=0.9.15` it compiles 0.9.15's.
  *Fresh clone: carla's build script emitted `carla_version=0.9.16` /
  `carla_version_0916`. The 0.9.15 override: step 1.*
- [x] `ros2 service call /carla/set_weather ... "{preset: ClearNoon}"` changes the sky, and it
  is still ClearNoon after a scenario that loads Town02.
  *Step 5: cloudiness 60 / rain 40 → ClearNoon; after `town02_episode_change` (passed)
  get_weather returned `preset='ClearNoon'`; the following Town01 load re-applied it too.*
- [x] `town01_traffic_light` passes on the rebuilt stack (regression).
  *Passed 2026-10-10 (failures=0) on rebuilt acb `ccfc64c` + csb, ego stack started with
  `just ego-av` (data_path check passed silently), weather ClearNoon.*

## Later

- `EnvironmentAction` → weather through the SSv2 fork (new protocol message), so a scenario
  can set its own sky.
