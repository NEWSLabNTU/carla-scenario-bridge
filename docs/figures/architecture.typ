// A scenario run: SSv2, a simulator (CARLA or Isaac Sim) and Autoware (roadmap 051's shape).
//   typst compile docs/figures/architecture.typ docs/figures/architecture.svg
#import "style.typ": *
#show: setup

#diagram(
  spacing: (8mm, 11mm),
  edge-stroke: 0.7pt,
  label-size: 6.5pt,

  // Scenario side
  b((1.5, 0), [*scenario_simulator_v2* \ runs the scenario, gives the verdict], scen, <ssv2>,
    width: 60mm),
  b((0.5, 1), [*scenario_world_bridge* \ SSv2's protocol in, world protocol out \
    knows no simulator], scen, <bridge>, width: 62mm),
  b((2.6, 1), [*agent relay* \ routes goals to each \ entity's agent], scen, <relay>),
  title((0.2, -0.55), [SCENARIO SIDE · ROS domain 9]),
  grp((0.2, -0.55), <ssv2>, <bridge>, <relay>),

  edge(<ssv2>, <bridge>, "-|>", [simulation_interface], label-side: right),
  edge(<ssv2>, <relay>, "-|>", [ego goals · engage], label-side: left),
  edge(<bridge>, <relay>, "-|>", [goals for `agent` entities], label-side: left),

  // World servers
  b((0, 2.3), [*swb_carla_world* \ load town · spawn \ spec physics · tick · lights],
    world, <cw>, width: 48mm),
  b((1.6, 2.3), [*swb_isaac_world* \ load USD · spawn \ step PhysX], world, <iw>, width: 48mm),
  b((1.6, 3.4), [*aib* \ the ego, from the vehicle spec \ sensors · status · /clock], veh, <aib>,
    width: 48mm),
  title((1.6, 1.85), [ISAAC SIM PROCESS]),
  grp((1.6, 1.85), <iw>, <aib>),
  edge(<bridge>, <cw>, "-|>", tag[world protocol], label-side: right, stroke: accent + 1pt),
  edge(<bridge>, <iw>, "-|>", tag[world protocol], label-side: left, stroke: accent + 1pt),
  edge(<iw>, <aib>, "-"),

  b((0, 3.4), [*CARLA server*], sim, <carla>, width: 48mm),
  edge(<cw>, <carla>, "-|>", [RPC], label-side: right),

  // Vehicle side
  b((0, 4.6), [*acb_bridge* \ attaches to the hero \ sensors · status · /clock], veh, <acb>,
    width: 48mm),
  b((2.6, 4.6), [*vehicle agent* \ sets the route, engages], veh, <agent>),
  edge(<carla>, <acb>, "-|>", [sensor data], label-side: left),
  edge(<relay>, <agent>, "-|>", [agent protocol, by entity], label-side: left),
  edge(<relay>, (4.3, 1), <bg>, "-|>"),

  b((1.3, 5.9), [*Autoware* \ autoware_sim_profile · tesla_model3 packages \
    the scenario's map], aw, <aw>, width: 70mm),
  edge(<acb>, <aw>, "<|-|>", tag[I5 contract], label-side: right, stroke: accent + 1pt),
  edge(<aib>, (1.6, 5.2), <aw>, "<|-|>", tag[I5 contract], label-side: left,
    stroke: accent + 1pt),
  edge(<agent>, <aw>, "-|>", [ADAPI], label-side: left),
  title((1.05, 4.1), [VEHICLE SIDE: ego · ROS domain 1]),
  grp((1.05, 4.1), <acb>, <agent>, <aw>),

  // One more vehicle side per entity whose controller is `agent`
  b((4.3, 4.6), [*bg_av_1* · ROS domain 2 \ acb_bridge attached to bg_av_1 \
    vehicle agent · Autoware], veh, <bg>, width: 50mm),
  title((4.3, 4.1), [MORE VEHICLE SIDES]),
  grp((4.3, 4.1), <bg>, (4.3, 5.3)),
  node((4.3, 5.3), text(size: 7pt, fill: gray.darken(45%), [one per Autoware instance]),
    stroke: none),
)
