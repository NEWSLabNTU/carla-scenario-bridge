# Tutorial 4: Demo run

Two complete Autoware stacks in one scenario, each driving its own CARLA vehicle, and a
screencast of it like the one in the [README](../../README.md). Builds on [Tutorial
1](01-first-run.md)'s stack.

## 1. Two Autoware vehicles

A vehicle in a scenario can be driven by an autopilot of its own instead of by SSv2: name its
controller `agent`. The scenario then reaches it exactly as it reaches the ego -- through the
relay, by entity name -- and you start one more vehicle side for it.
`multi_av/town01_two_av.xosc` has the ego and a background AV, `bg_av_1`:

```xml
<ScenarioObject name="bg_av_1">
  <Vehicle name="vehicle.tesla.model3" vehicleCategory="car"> ... </Vehicle>
  <ObjectController><Controller name="agent"><Properties /></Controller></ObjectController>
</ScenarioObject>
```

Both get a `TeleportAction` and an `AcquirePositionAction` (their goals) in `Init`, and the
scenario passes when both have arrived. The bridge spawns `bg_av_1` as a physics vehicle with
that role name, so its vehicle side finds it in CARLA.

Keep Terminals 1-3 of Tutorial 1 running and start the second vehicle side, in a ROS domain
of its own:

```bash
# Terminal 3b
ROS_DOMAIN_ID=2 ros2 launch csb_launch background_av.launch.xml \
    entity:=bg_av_1 map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560
```

`background_av.launch.xml` is the same Autoware vehicle side with lighter perception choices
(CPU lidar clustering, no traffic light camera), so a second stack fits beside CARLA and the
ego's on one GPU. Ready when Terminal 2's relay prints
`entity 'bg_av_1': agent acb_agent registered`. Then:

```bash
# Terminal 4
ROS_DOMAIN_ID=9 ros2 launch csb_launch scenario.launch.xml \
    scenario:=$(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/multi_av/town01_two_av.xosc \
    output_directory:=/tmp/two_av
grep -o 'failures="[0-9]*" errors="[0-9]*"' /tmp/two_av/scenario_test_runner/result.junit.xml | head -1
# failures="0" errors="0"
```

Each Autoware perceives the other through CARLA's sensors: real, Autoware-driven traffic,
not an SSv2 puppet. More vehicles: one `agent` entity and one vehicle side (its own domain,
`entity:=<name>`) each, as long as the machine holds them.

## 2. Windows for the demo

The README's showreel shows three windows: CARLA from behind the ego, and each vehicle's
RViz. All three come with the workspace.

**A chase camera** on the ego (the CARLA vehicle named `hero`), in a window of its own:

```bash
SDL_VIDEO_WINDOW_POS=70,28 VIEW_W=740 VIEW_H=416 ros2 run csb_launch chase_view
```

It is a free camera moved behind the vehicle every frame, not attached to it, so a scenario
ending never leaves a sensor behind; it waits for the next ego when there is none.

**RViz layouts** that place the two Autoware windows on the right half of a 1920x1080 screen
(ego top, `bg_av_1` bottom), each following its own vehicle from behind. Restart the vehicle
sides with them:

```bash
RVIZ=$(ros2 pkg prefix --share csb_launch)/rviz
ROS_DOMAIN_ID=1 ros2 launch acb_launch carla_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 vehicle_name:=hero relay:=tcp://localhost:5560 \
    rviz_config:=$RVIZ/ego_autoware.rviz
ROS_DOMAIN_ID=2 ros2 launch csb_launch background_av.launch.xml \
    entity:=bg_av_1 map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560 \
    launch_rviz:=true rviz_config:=$RVIZ/bg_av_1_autoware.rviz
```

For a single vehicle, `autoware_cast.rviz` is the same view over the whole right half.

## 3. Record

Any screen recorder works. With ffmpeg, the whole X display, stopped with `q` when the
scenario has ended:

```bash
ffmpeg -f x11grab -framerate 15 -video_size 1920x1080 -i $DISPLAY \
    -vf scale=1280:720 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p demo.mp4
```

Start the recording, then the scenario (Terminal 4) in a terminal at the bottom left. Two
Autoware stacks and two RViz windows load a machine heavily; a recorded run is a demo, not a
measurement of Autoware's performance.

**A GIF** for a README or a slide: `scripts/make_showreel.sh` in the repository cuts the
showreel this way -- crops each window out of the 1280x720 recording, stacks them, speeds the
whole up 6-8x and writes a palette GIF. Its crop rectangles are for the layout above recorded as
above (1920x1080 screen, scaled to 1280x720); adjust them for your own.

## Where next

- [User Guide](../user-guide.md) -- every option of the launches, the CARLA world (weather),
  your own Autoware, timeouts, troubleshooting.
- [Writing scenarios](../design/scenario-authoring.md) -- vehicles CARLA's Traffic Manager
  drives (`simulator_autopilot`), traffic lights, collisions.
- [Developer Guide](../development.md) -- if you want to change the bridge itself.
