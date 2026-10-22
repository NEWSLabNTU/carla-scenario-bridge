"""Phase 036 smoke test: start Isaac Sim headless, build a world, step it, report timings."""
import os
import time

t0 = time.time()
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not os.environ.get("DISPLAY")})  # windowed on a display (VNC), else headless
t_app = time.time() - t0

from isaacsim.core.api import World  # noqa: E402
import omni.kit.app  # noqa: E402

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane()
world.reset()
t1 = time.time()
for _ in range(600):
    world.step(render=True)
t_step = (time.time() - t1) / 600
ver = omni.kit.app.get_app().get_build_version()
print(f"SMOKE OK kit {ver}; app start {t_app:.1f} s; step+render {1e3 * t_step:.2f} ms; sim time {world.current_time:.2f} s")
app.close()
