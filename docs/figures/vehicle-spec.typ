// One vehicle spec feeds both simulators, SSv2 and Autoware, and checks hold them to it.
//   typst compile docs/figures/vehicle-spec.typ docs/figures/vehicle-spec.svg
#import "style.typ": *
#show: setup

#diagram(
  spacing: (7mm, 10mm),
  edge-stroke: 0.7pt,
  label-size: 6.5pt,

  // sim_vehicle_spec: the car, for any autopilot
  title((1, -0.5), [SIM_VEHICLE_SPEC · NO AUTOPILOT]),
  b((2.5, 0), [*vehicles/tesla_model3.yaml* \ geometry · inertia · tires · brakes \
    powertrain · steering · sensors \ + isaac: and carla: sections], spec, <spec>,
    width: 62mm),
  b((1, 1.3), [*Isaac rig* \ for aib], veh, <rig>, width: 36mm),
  b((2, 1.3), [*carla_physics.yaml* \ mass · tires · torque \ applied at spawn], world,
    <phys>, width: 36mm),
  b((3, 1.3), [*carla_sensors.yaml* \ for acb_bridge], veh, <sens>, width: 36mm),
  b((4, 1.3), [*SSv2 catalog* \ box · axles \ performance], scen, <cat>, width: 36mm),
  grp((1, -0.5), <spec>, <rig>, <phys>, <sens>, <cat>),
  for o in (<rig>, <phys>, <sens>, <cat>) {
    edge(<spec>, o, "-|>", stroke: accent + 0.9pt)
  },

  // autoware_sim_vehicle: the same car, for Autoware
  title((2.5, 2.25), [AUTOWARE_SIM_VEHICLE]),
  b((2, 2.8), [*pedal maps* \ measured on CARLA, tagged \ with the physics hash], aw, <pm>,
    width: 46mm),
  b((3, 2.8), [*Autoware packages* \ vehicle_info · URDF \ sensor kit · profile], aw, <awp>,
    width: 40mm),
  grp((2.5, 2.25), <pm>, <awp>),
  edge(<phys>, <pm>, "-|>", [calibration run], label-side: right),
  edge(<spec>, (2.5, 2.15), (3, 2.15), <awp>, "-|>", [generates], label-side: left,
    stroke: accent + 0.9pt),
  node((4.3, 2.8), text(size: 7pt, fill: gray.darken(45%), [another autopilot: \
    its own `<autopilot>_sim_vehicle`, \ from the same spec]), stroke: (paint: gray,
    dash: "dotted", thickness: 0.6pt), inset: 5pt, corner-radius: 3pt),

  // Checks
  title((0.5, 3.75), [CHECKS: THE SAME CAR EVERYWHERE]),
  b((0.5, 4.3), [*build* \ outputs regenerated and compared; \ no ego box typed in a scenario],
    sim, <c1>, width: 60mm),
  b((2, 4.3), [*start* \ every part reports its spec_hash; \ a mismatch is refused],
    sim, <c2>, width: 60mm),
  b((3.5, 4.3), [*spawn* \ box and wheels against the catalog; \ CARLA physics read back],
    sim, <c3>, width: 60mm),
  b((5, 4.3), [*drive* \ vehicle_check: acceleration, \ braking, steering per simulator],
    sim, <c4>, width: 60mm),
  grp((0.5, 3.75), <c1>, <c2>, <c3>, <c4>),
)
