// Autoware on a simulator without SSv2: a scene set up by hand in CARLA, or Isaac's runner.
//   typst compile docs/figures/standalone.typ docs/figures/standalone.svg
#import "style.typ": *
#show: setup

#diagram(
  spacing: (8mm, 10mm),
  edge-stroke: 0.7pt,
  label-size: 6.5pt,

  b((0, 0), [*your CARLA client* \ spawns the hero \ traffic, weather], sim, <client>),
  b((1, 0), [*CARLA server* \ lights cycling], sim, <carla>),
  edge(<client>, <carla>, "-|>", [RPC], label-side: left),
  b((1, 1.2), [*acb_bridge* \ makes the hero the spec's car \ sensors · status · /clock], veh,
    <acb>, width: 52mm),
  edge(<carla>, <acb>, "<|-|>", [RPC · sensor data], label-side: left),
  b((1, 2.5), [*Autoware* \ pose and goal from RViz], aw, <aw1>),
  edge(<acb>, <aw1>, "<|-|>", tag[I5 contract], label-side: left, stroke: accent + 1pt),
  title((0, -0.55), [CARLA · A SCENE SET UP BY HAND]),
  grp((0, -0.55), <client>, <carla>, <acb>, <aw1>),

  b((2.5, 1.2), [*aib runner* \ Isaac Sim with the ego \ from the vehicle spec], veh, <runner>),
  b((2.5, 2.5), [*Autoware* \ the same packages], aw, <aw2>),
  edge(<runner>, <aw2>, "<|-|>", tag[I5 contract], label-side: left, stroke: accent + 1pt),
  title((2.5, 0.65), [ISAAC SIM]),
  grp((2.5, 0.65), <runner>, <aw2>),
)
