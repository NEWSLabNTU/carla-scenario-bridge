// One vehicle spec feeds Autoware, both simulators and SSv2, and checks hold them to it.
//   typst compile docs/figures/vehicle-spec.typ docs/figures/vehicle-spec.svg
#import "style.typ": *
#show: setup

#diagram(
  spacing: (5mm, 10mm),
  edge-stroke: 0.7pt,
  label-size: 6.5pt,

  b((2, 0), [*vehicles/tesla_model3.yaml* \ geometry · inertia · tires · brakes \
    powertrain · steering · sensors \ + isaac: and carla: sections], spec, <spec>,
    width: 62mm),

  b((0, 1.4), [*Autoware packages* \ vehicle_info · URDF \ sensor kit], aw, <o1>, width: 36mm),
  b((1, 1.4), [*Isaac rig* \ for aib], veh, <o2>, width: 36mm),
  b((2, 1.4), [*carla_physics.yaml* \ mass · tires · torque \ applied at spawn], world, <o3>,
    width: 36mm),
  b((3, 1.4), [*carla_sensors.yaml* \ for acb_bridge], veh, <o4>, width: 36mm),
  b((4, 1.4), [*SSv2 catalog* \ box · axles \ performance], scen, <o5>, width: 36mm),
  for o in (<o1>, <o2>, <o3>, <o4>, <o5>) {
    edge(<spec>, o, "-|>", stroke: accent + 0.9pt)
  },
  b((2, 2.5), [*pedal maps* \ measured on CARLA \ tagged with the physics hash \
    for Autoware and acb_bridge], spec, <pm>, width: 56mm),
  edge(<o3>, <pm>, "-|>", [calibration run], label-side: left),

  b((0.5, 3.8), [*build* \ outputs regenerated and compared; \ no scenario types its own ego box],
    sim, <c1>, width: 56mm),
  b((2, 3.8), [*spawn* \ box and wheels against the catalog \ CARLA physics read back],
    sim, <c2>, width: 56mm),
  b((3.5, 3.8), [*drive* \ vehicle_check: acceleration, braking \ and steering on each simulator],
    sim, <c3>, width: 56mm),
  title((0.5, 3.3), [CHECKS: THE SAME CAR EVERYWHERE]),
  grp((0.5, 3.3), <c1>, <c2>, <c3>),
)
