# Phase 020 proof screenshots (steps 1-5)

Taken 2026-10-10 on the development host with the stack as the user guide describes it:
CARLA 0.9.16 (Town01/Town02), the simulation side (`simulation.launch.xml`, domain 9), the
ego's vehicle side (`just ego-av`, domain 1) and `ros2 run csb_launch run_suite` over the
installed `csb_examples` scenarios. Viewers ran on a local VNC display
(`vncserver :5 -geometry 1920x1080 -SecurityTypes None -localhost`): Autoware's RViz
(`autoware.rviz`, view following `base_link`) in the ego's domain and an xterm tailing
`run_suite`'s output. Captured with `import -window root -display :5`, halved.

| File | Shows |
| --- | --- |
| `01_basic_town01_pedestrian_stopped.jpg` | `basic/town01_pedestrian.xosc`, mid-run on Town01: the ego engaged (AUTONOMOUS, routing set), stopped at 0 km/h behind an `obstacle_stop` wall for the pedestrian crossing its lane; run_suite at scenario 3 of 7. |
| `02_basic_town01_traffic_light_red_ahead.jpg` | `basic/town01_traffic_light.xosc`, mid-run on Town01: the ego driving at 15 km/h toward the junction; the HUD's signal icon is red (the SSv2-commanded red reached Autoware from CARLA's light); run_suite at scenario 6 of 7, the first five PASS. |
| `03_basic_town02_episode_change_carla.jpg` | `basic/town02_episode_change.xosc`: CARLA after the bridge loaded Town02 (overhead RGB camera at a Town02 spawn point, from a one-shot camera spawned on the new episode). This scenario has no entities and the ego stack is configured for Town01, so there is no ego to show in RViz here; it passes when 5 s of simulation time have run, and the next scenario loads Town01 back. |
| `04_awf_acc_following_lead_vehicle.jpg` | AWF `UC-ACC-001-0001` (a variant mid-run, Town01 lanelet 5230): the ego at 14 km/h with its trajectory slowing toward the preceding vehicle (`obstacle_stop` marker on the NPC ahead). All 27 variants passed. |
| `05_awf_aeb_stopped_for_pedestrian.jpg` | AWF `UC-AEB-001-0001` (a variant mid-run): the ego stopped at 0 km/h short of the standing pedestrian, `obstacle_stop` wall in front of it. All 9 variants passed. |
| `06_basic_run_suite_summary.jpg` | The whole display at the end of `run_suite` over `basic/`: 7/7 scenarios passed, RViz with the ego back on Town01. |
| `07_basic_run_suite_summary_terminal.jpg` | The run_suite terminal from 06 at full resolution: one PASS line per scenario with its time, and the `suite.junit.xml` path. |

The GNOME notification at the top of the RViz captures belongs to the VNC desktop, not to
the stack.
