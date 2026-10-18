// Target SSv2 + Isaac Sim + Autoware stack (roadmap 038, 043, 044, 039).
// Fill = stage: done (built), 1 = single-vehicle E2E (043), 2 = NPCs and traffic lights
// (044), 3 = several vehicles, V2X and the gap campaign (039).
//   typst compile docs/design/isaac-stack.typ docs/design/isaac-stack.svg
#import "@preview/fletcher:0.5.8" as fletcher: diagram, node, edge

#set page(width: auto, height: auto, margin: 8mm, fill: white)
#set text(font: "DejaVu Sans", size: 8pt)

#let done = rgb("#d9ead3")
#let s1 = rgb("#cfe2f3")
#let s2 = rgb("#fff2cc")
#let s3 = rgb("#f4cccc")
#let grp = (paint: gray.darken(20%), dash: "dashed", thickness: 0.8pt)
#let b(pos, body, fill, name) = node(pos, body, fill: fill, name: name, width: 40mm,
  corner-radius: 3pt, stroke: 0.6pt + fill.darken(45%))
// A group title: as wide as a box for the layout, its text running on to the right.
#let title(pos, body) = node(pos, place(left + horizon, dx: -2mm, dy: -1mm,
  box(width: 120mm, text(size: 9pt, weight: "bold", body))), stroke: none, width: 40mm,
  height: 4mm)

#diagram(
  spacing: (6mm, 9mm),
  edge-stroke: 0.7pt,
  label-size: 7pt,

  // ── Scenario side and tooling ────────────────────────────────────────────────
  title((0, 0.4), [Scenario domain 9]),
  b((0, 1), [*SSv2* \ scenario_test_runner \ openscenario_interpreter \ fuzzed .xosc, AWF yaml],
    done, <ssv2>),
  b((1, 1), [*just scenario / acceptance / e2e* \ backend-aware (Isaac too)], s1, <tool>),
  b((1, 2), [*Parallel pipelines* \ world server per ISAAC_PORT \ + domain set (campaign)],
    s3, <par>),

  // ── Simulation launch ────────────────────────────────────────────────────────
  title((0, 3.4), [Simulation launch (play_launch :8084)]),
  b((0, 4), [*csb bridge* · IsaacBackend \ ego · scenario NPCs · walkers], done, <br>),
  b((0, 5), [UpdateTrafficLights → SetSignals], s2, <tls>),
  b((1, 5), [Agent-driven vehicles \ (agent link)], s3, <agl>),
  b((0, 6), [Gap trace → oracle, \ campaign runner], s3, <gt>),
  b((1, 6), [World server under play_launch \ respawn · logs · health], s1, <wsup>),
  b((1, 10), [*Agent relay* :5560], done, <rl>),
  node(enclose: ((0, 3.4), <br>, <tls>, <agl>, <gt>, <wsup>, <rl>), stroke: grp, inset: 6pt,
    name: <sim>),

  // ── Offline asset pipeline ───────────────────────────────────────────────────
  title((2, 0.4), [Offline asset pipeline (once per town / car)]),
  b((2, 1), [*CARLA 0.9.16 editor* \ -run=SimReadyExport], done, <ce>),
  b((3, 1), [*Town0X.usd* \ 01 done · 02/03/05/10 to do], s1, <tu>),
  b((4, 1), [*Physics layer script* \ → Town0X_physics.usda], s1, <pl>),
  b((5, 1), [*csb_isaac.usd_world* \ → isaac/world.yaml \ same frame as Autoware map], done,
    <uw>),
  b((2, 2), [*Signal table* \ lanelet TL id ↔ lamp prims \ (from OpenDRIVE signals)], s2, <st>),
  b((3, 2), [*vehicle.\*.usd* ×41 \ + vehicles.json], done, <vu>),
  b((4, 2), [*Blueprint physics* \ mass · wheelbase · steer], s1, <bp>),
  b((5, 2), [*acb vehicle_info* \ (must agree)], s1, <vi>),
  node(enclose: ((2, 0.4), <ce>, <uw>, <st>, <vi>), stroke: grp, inset: 6pt, name: <off>),
  edge(<ce>, <tu>, "-|>"),
  edge(<tu>, <pl>, "-|>"),
  edge(<pl>, <uw>, "-|>"),
  edge(<ce>, <st>, "-|>"),
  edge(<ce>, (2.5, 1), (2.5, 2), <vu>, "-|>"),
  edge(<vu>, <bp>, "-|>"),
  edge(<bp>, <vi>, "<|-|>", stroke: (dash: "dashed")),

  // ── Isaac world server ───────────────────────────────────────────────────────
  title((2, 3.4), [Isaac world server (Isaac Sim 6.1, py3.12)]),
  b((2, 4), [*Lockstep* 0.05 s = 3 × 1/60 PhysX \ /clock (wall offset)], done, <ls>),
  b((3, 4), [*World*: CARLA town USD + colliders \ reload on map change], s1, <wd>),
  b((4, 4), [*Signal lamps* from SetSignals], s2, <sg>),
  b((5, 4), [*Viewport* VNC :8 \ GNOME + VirtualGL, or headless], done, <vp>),
  b((2, 5), [*Ego*: PhysX drive-none \ CARLA body], done, <ego>),
  b((3, 5), [Ego physics from \ CARLA blueprint], s1, <egop>),
  b((4, 5), [N egos, one per domain], s3, <egon>),
  b((2, 6), [*NPCs*: kinematic], done, <npck>),
  b((3, 6), [NPCs: dynamic, \ tracking SSv2 pose], s2, <npcd>),
  b((4, 6), [Walkers: CARLA meshes], s2, <wk>),
  title((2, 6.6), [Per-ego ROS context (EgoRos, domain d)]),
  b((2, 7.2), [pub: lidar OS1-128 (1 frame late) \ IMU · GNSS · /vehicle/status/\* \
    sub: control_cmd · gear · turn ind.], done, <sen>),
  b((3, 7.2), [pub: traffic_signals \ (ground truth, as acb on CARLA)], s2, <tlp>),
  node(enclose: ((2, 6.6), <sen>, <tlp>), stroke: grp, inset: 4pt, name: <ros>),
  node(enclose: ((2, 3.4), <ls>, <vp>, <npck>, <ros>), stroke: grp, inset: 6pt, name: <isa>),

  // ── Ego stacks ───────────────────────────────────────────────────────────────
  title((2, 8.6), [Ego stack, domain 1 (play_launch :8082)]),
  b((2, 9.2), [*Autoware 1.5.0* + overlay \ acb carla_simulator.launch.xml \
    launch_vehicle_interface:=false], done, <aw>),
  b((3, 9.2), [Traffic-light consumer], s2, <tlc>),
  b((2, 10), [*acb_pilot* \ initialize timeout 60 s], s1, <pil>),
  node(enclose: ((2, 8.6), <aw>, <tlc>, <pil>), stroke: grp, inset: 6pt, name: <eg1>),
  title((5, 8.6), [BG ego stacks (2..N)]),
  b((5, 9.2), [*Autoware + acb_pilot* \ one per domain (just bg-isaac)], s3, <awn>),
  node(enclose: ((5, 8.6), <awn>), stroke: grp, inset: 6pt, name: <egn>),
  b((4, 10), [*V2X* between egos], s3, <v2x>),

  // ── Flows ────────────────────────────────────────────────────────────────────
  edge(<ssv2>, <br>, "<|-|>", [ZMQ :5555 \ SSv2 protocol], label-side: left),
  edge(<br>, <ls>, "<|-|>", [csb_world.proto \ :ISAAC_PORT]),
  edge(<wsup>, <isa>, "-|>", stroke: (dash: "dotted")),
  edge(<off>, <isa>, "-|>", [worlds, vehicles, \ signal table, ego params], label-side: left),
  edge(<sen>, <aw>, "<|-|>", [DDS domain 1], label-side: left),
  edge(<tlp>, <tlc>, "-|>"),
  edge(<ros>, <awn>, "-|>", [DDS domains 2..N], label-side: left),
  edge(<pil>, <rl>, "-|>", [TCP]),
  edge(<awn>, (5, 10.8), (1, 10.8), <rl>, "-|>", [TCP]),
  edge(<aw>, <v2x>, "<|-|>"),
  edge(<awn>, <v2x>, "<|-|>"),

  // ── Legend ───────────────────────────────────────────────────────────────────
  b((2, 11.6), [Built], done, <l0>),
  b((3, 11.6), [Stage 1 · single-vehicle E2E (043)], s1, <l1>),
  b((4, 11.6), [Stage 2 · NPCs, traffic lights (044)], s2, <l2>),
  b((5, 11.6), [Stage 3 · multi-vehicle, V2X, campaign (039)], s3, <l3>),
)
