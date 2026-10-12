# Tutorial 3: Run and judge

Run many scenarios as one suite, add CARLA's own collision check to SSv2's verdict, and find
your way around the results. Uses the stack from [Tutorial 1](01-first-run.md) and the
scenario from [Tutorial 2](02-prepare-a-scenario.md).

## 1. A suite

`run_suite` runs scenarios one after another -- each through `scenario.launch.xml`, in its
own output directory -- and merges the verdicts into one JUnit file:

```bash
ROS_DOMAIN_ID=9 ros2 run csb_launch run_suite \
    $(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/basic \
    ~/my_scenarios \
    --output ~/csb_results/today
```

Arguments are scenario files (`.xosc`, `.yaml`) or directories (every scenario file in
them, sorted, recursively). It prints one line per scenario and a total:

```
[run_suite] 8 scenario(s), domain 9, results in /home/you/csb_results/today
[run_suite] (1/8) town01_ego_drive: .../basic/town01_ego_drive.xosc
[run_suite] PASS town01_ego_drive: 1/1 passed in 41 s
...
[run_suite] 8/8 scenario(s) passed; /home/you/csb_results/today/suite.junit.xml
```

Before each scenario it asks the relay whether the scenario's vehicles are registered, and
fails that scenario at once if one is not (`Preflight: no vehicle agent registered ...`)
instead of letting it time out. If play_launch is installed it runs each scenario with it;
otherwise with `ros2 launch`.

## 2. CARLA's contacts in the verdict

SSv2 judges collisions in its own world: the declared boxes, in 2D, and nothing static. The
ego can clip a pole, a curb, or an NPC whose CARLA body is bigger than its declared box, and
SSv2 still passes the run. The bridge records what CARLA's collision sensor saw for every
run; `--physical-verdict` puts it next to SSv2's verdict:

```bash
ROS_DOMAIN_ID=9 ros2 run csb_launch run_suite ~/my_scenarios \
    --output ~/csb_results/physical --physical-verdict
```

Each scenario now has two testcases, SSv2's and `carla_contact`:

```
[run_suite] PASS follow_to_goal: 2/2 passed in 52 s
```

| `carla_contact` | When |
|---|---|
| pass | no contact, or only contacts the scenario declares |
| failure | CARLA saw the ego touch something else; the message names it, the frame and the impulse |
| error | no contact report for the run (bridge too old, reports turned off) |

A scenario that is *meant* to make contact declares with whom (SSv2 ignores the parameter):

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string" value="npc_rear" />
```

A comma-separated list of entity names; `static` allows contact with the static world. The
starter `town01_rear_contact` is an example. When the two testcases disagree -- SSv2 passed,
`carla_contact` failed -- the scenario passed only because SSv2 did not see what CARLA did.

## 3. Bodies that do not fit

The bridge gives every NPC the CARLA body closest in size to its declared box. When CARLA has
nothing close -- a 12 m truck, the longest CARLA truck is 8.5 m -- the `carla_contact`
testcase's output lists it, without failing anything:

```
body outside tolerance: 'Npc1' declared 4.00 x 1.80 x 2.50 m, spawned as
'vehicle.jeep.wrangler_rubicon' 3.87 x 1.91 x 1.88 m (difference -0.13 / +0.11 / -0.62 m)
```

It tells you where the two worlds' geometry differ. Set `model3d` on the entity to choose the
body yourself.

## 4. The results

```
~/csb_results/physical/
  suite.junit.xml                       every scenario, merged: open it in any JUnit viewer
  follow_to_goal/
    launch.log                          the scenario launch's console
    scenario_test_runner/
      result.junit.xml                  SSv2's verdict for this scenario
      execution_timer.json              how long SSv2's frames took
      follow_to_goal/                   SSv2's per-run files
```

`suite.junit.xml` is what a CI job reads. The contact reports themselves are JSON under
`~/.ros/csb/contacts/`, one per run; `ros2 run csb_launch physical_verdict --help` judges a
single run's result outside a suite.

## 5. Scenarios with variants

SSv2's YAML scenarios can sweep parameters (`ScenarioModifiers`): one file, one run per
combination. The AWF use cases do this:

```bash
ROS_DOMAIN_ID=9 ros2 run csb_launch run_suite \
    $(ros2 pkg prefix csb_examples)/share/csb_examples/scenarios/awf/UC-AEB-001-0001_Standing-pedestrian.yaml \
    --output ~/csb_results/aeb --physical-verdict
# [run_suite] PASS UC-AEB-001-0001_Standing-pedestrian: 10/10 passed in 537 s
```

Nine variants (ego speed x pedestrian distance) plus the contact check. SSv2 bounds each
variant's time itself; `--timeout-per-scenario S` caps a whole file if you need a hard limit.

Next: [Tutorial 4, demo run](04-demo-run.md).
