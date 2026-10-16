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

- [ ] Install Isaac Sim 6.1 under `/mnt/disk1/isaac-sim/`; `just isaac-check` recipe prints
      the version and runs a 10 s headless smoke test.
- [ ] VNC display recipe (`just isaac-vnc`), documented next to the CARLA display notes.
- [ ] Spike scripts in `csb_isaac/spikes/` (kept, small): domains, Cyclone, lockstep, cost.
- [ ] Record results in this doc's Outcome section; update the design doc where a fallback
      was taken.

## Acceptance

Every question above answered with a measurement or a pass/fail on this host, and a decision
recorded for each: domain mechanism (contexts or rclpy), RMW, step model, the number of ego
sensor sets one 5090 holds at 10 Hz real time.
