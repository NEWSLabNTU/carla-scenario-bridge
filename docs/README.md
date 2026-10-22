# Documentation

## Using scenario-world-bridge

- [README](../README.md) -- what it is, prerequisites, setup, quick start
- Tutorials, in order:
  1. [First run](tutorials/01-first-run.md) -- both sides up, a starter scenario, its verdict
  2. [Prepare a scenario](tutorials/02-prepare-a-scenario.md) -- poses on the map, a `.xosc` of your own
  3. [Run and judge](tutorials/03-run-and-judge.md) -- suites, CARLA's contacts in the verdict, results
  4. [Demo run](tutorials/04-demo-run.md) -- two Autoware vehicles, recording a screencast
- [User Guide](user-guide.md) -- the reference: install, maps, scenario directories, running,
  the CARLA world, troubleshooting
- [Writing an agent](agent-interface.md) -- drive scenarios with your own autopilot: the
  `swb_agent.Agent` class, `ros_adapter` from YAML, `swb-agent check` / `new`
- [Writing scenarios](design/scenario-authoring.md) -- what a scenario may use on CARLA, and how
- [User workflow and the vehicle agent](design/user-workflow.md) -- the two sides, swapping
  CARLA or Autoware

## Developing scenario-world-bridge

- [Developer Guide](development.md) -- build, test, layout, how a scenario runs, conventions,
  and the index of design documents ([design/](design/))

## Roadmap

- [Roadmap](roadmap/README.md) -- one document per phase: problem, design, steps, measured results
