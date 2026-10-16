# Phase 036: Isaac Sim Bring-up and the Domain Spike

Install Isaac Sim on this host, run it on a VNC display and headless, and answer the
questions the rest of the Isaac work depends on before writing any of it
([design/simulator-backends.md](../design/simulator-backends.md) "Risks").

**Source**: user direction 2026-10-16 (Isaac Sim for a sponsor's off-road competition; "you
can use vncserver" for Isaac).
**Depends on**: nothing (can run alongside 035).
**Feeds**: [037](037-isaac-world-server.md), [038](038-isaac-vehicle-bridge.md),
[039](039-isaac-multi-autoware.md).

## Host

RTX 5090 (32 GB), driver 580.105.08 (the off-road WG simulator asks for 580+), Ubuntu 22.04,
ROS 2 Humble. `/home` has ~250 GB free and is the disk other users saturate (CLAUDE.md "The
log disk can stall a node"): Isaac, its caches and its logs go on `/mnt/disk1`
(`/mnt/disk1/isaac-sim/`). Display: TurboVNC (`/opt/TurboVNC/bin/vncserver`) on a private
display, Vulkan pinned to the NVIDIA ICD as for CARLA
(`VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`).

## Questions and how each is answered

1. **Install.** Isaac Sim 6.1 (pip wheels or the standalone package; whichever runs
   `python.sh` scripts and the ROS 2 bridge against the system Humble). Record the exact
   version, install command, first-launch shader-cache time, and where caches live.
2. **Several ROS domains in one process.** Two vehicles in one stage, each with its own
   ROS 2 action graph whose `ROS2 Context` has `useDomainIDEnvVar=false` and `domain_id` 1 and 2,
   each publishing `/clock`, an IMU and a lidar. Pass: `ROS_DOMAIN_ID=1 ros2 topic list` shows
   only vehicle 1's topics, domain 2 only vehicle 2's, rates as configured.
   Fallback if it fails: rclpy contexts created by a standalone script (`rclpy.Context`,
   `init(domain_id=k)`), one per vehicle, publishing data read from the render products; same
   pass test.
3. **Cyclone DDS with our localhost config.** Same test with
   `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI=file://.../config/cyclonedds-localhost.xml`
   (a forum report has the 5.1 bridge failing to load under Cyclone with `ROS_DOMAIN_ID` set).
   An Autoware node on domain 1 must see vehicle 1's topics.
4. **Lockstep.** A standalone script that waits on a socket, steps physics n times and renders
   once, then replies: is the step deterministic for a given input, and does
   rendering-on-demand keep sensor timestamps on the sim clock?
5. **Cost.** Frame time for one ego sensor set (the lidars, cameras, IMU and GNSS of
   `acb_sensor_kit_description`, at the rates and resolutions acb runs them on CARLA) on an
   empty world and on a large mesh world; then two and four sets. GPU memory per set. This
   sizes 039.
6. **Headless vs VNC.** Both start from the same script; headless for runs, VNC for looking.

## Steps

- [x] Install Isaac Sim 6.1 under `/mnt/disk1/isaac-sim/`
      (`src/csb_isaac/scripts/install_isaac.sh`); `just isaac-check` prints the version and runs
      a 10 s headless smoke test.
- [x] VNC display recipe (`just isaac-vnc`: TurboVNC `:8`, localhost only, like the existing
      desktops).
- [x] Spike scripts in `src/csb_isaac/spikes/`: `smoke.py`, `domains.py` + `check_domains.sh`,
      `lockstep.py`, `cost.py`, run with `run_spike.sh <rmw> <log> <spike.py>`.
- [x] Results below; the design doc's fallback was not needed.

## Acceptance

Every question above answered with a measurement or a pass/fail on this host, and a decision
recorded for each: domain mechanism (contexts or rclpy), RMW, step model, the number of ego
sensor sets one 5090 holds at 10 Hz real time.

## Outcome (2026-10-17)

All six questions answered on this host; no fallback needed.

1. **Install.** `isaacsim[all,extscache]==6.1.0.0` from pypi.nvidia.com into a Python 3.12
   venv made by uv (the host has 3.10), PyTorch 2.11 cu128 first; uv needs
   `--prerelease=allow` (a pinned `tinyobjloader` release candidate). 27 GB. Kit 110.3.0.
   Runs on driver 580.105.08 although NVIDIA tested 595. Kit's caches
   (`~/.cache/ov`, `~/.local/share/ov`, `~/.nvidia-omniverse`) are symlinks into
   `/mnt/disk1/isaac-sim/`. First start 32 s (shader compiles), then 5.5 s; an empty world
   steps and renders in 1.8-2.6 ms. Isaac bundles ROS 2 Humble libraries built for its
   Python 3.12 (rclpy, rmw_fastrtps_cpp, rmw_cyclonedds_cpp); `isaac_env.sh` puts them on the
   library path. The system Humble (Python 3.10) is not sourced into Isaac.
2. **Several ROS domains in one process: yes, by context.** Two vehicles, one action graph each,
   `ROS2Context` with `useDomainIDEnvVar=false` and `domain_id` 1 and 2. A system-Humble client
   on domain 1 sees only `vehicle_1` on `/tf` and its `/clock`, domain 2 only `vehicle_2`,
   domain 3 nothing. The rclpy fallback is not needed.
3. **DDS.** Works with Cyclone under the stack's `config/cyclonedds-localhost.xml` and under
   Autoware's `cyclonedds.xml`, and with Fast DDS; the forum's 5.1 Cyclone failure does not
   reproduce on 6.1. Both sides must run the same RMW (an Isaac on Cyclone and a client on
   Fast DDS saw nothing).
4. **Lockstep: deterministic.** A ZMQ REP loop that sets 20 kinematic NPC poses, steps 6
   physics substeps (60 Hz, a 10 Hz frame) and renders once: two fresh servers give
   bitwise-identical states for a dynamic body (and rendering or not does not change the
   physics). Round trip 6.8 ms median, 8.0 ms p95 with rendering; 4.3 / 5.6 ms without.
   Rendering-on-demand keeps sensor stamps on the sim clock (`/clock` and `/tf` carry sim
   time). Free-running, Isaac's sim time runs about 9× real time on an empty world, so the
   pacing has to come from csb, as with CARLA in synchronous mode.
5. **Cost of a sensor set, linear.** One Ouster OS1 REV7 128-channel 20 Hz RTX lidar
   (2.6 M points/s, acb's rate; about 30 k returns per scan in a box field, full scans) plus
   one 1280×720 RTX camera at 10 Hz, each published over ROS 2 on its own domain, in a field
   of 200 boxes, physics 60 Hz, render 20 Hz:

   | Sets | ms per 20 Hz frame (median / p95) | GPU for the scene |
   |---|---|---|
   | 1 | 7.8 / 9.3 | 0.9 GB |
   | 2 | 12.8 / 13.4 | 1.2 GB |
   | 4 | 22.5 / 24.0 | 1.7 GB |
   | 8 | 41.2 / 43.3 | 2.8 GB |

   About 5 ms and 0.26 GB per set; real time is 50 ms per frame, so one 5090 holds eight sets
   on this world. A large mesh world (040) will cost more per render; remeasure there.
6. **Headless and VNC.** The same scripts run headless (no `DISPLAY`) or windowed on the VNC
   display (`DISPLAY=:8`): first windowed start 144 s (the full UI), then a 1920×1080 viewport
   at 22 ms per step.

**Decisions**: per-ego `ROS2Context` with an explicit `domain_id`; Cyclone with the stack's
localhost config, set the same on both sides; the world server owns the loop and steps on
request (029's design holds); budget at least 5 ms per full sensor set per 20 Hz frame, so up
to eight egos per 5090 on a light world.
