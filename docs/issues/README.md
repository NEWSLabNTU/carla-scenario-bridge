# Issues

One file per defect: what is wrong, why it matters, how it was confirmed, and the fix or
what is still open.

| # | Issue | Severity | Status |
|---|---|---|---|
| [001](001-soil-tile-rebuild-memory.md) | Each soil tile rebuild left cooked collision in the Isaac world server (~0.4 MB, then 64 KB); now no mesh is cooked after load | low | fixed |
| [002](002-mid-sand-mrm.md) | Emergency stops on town01_soil: NDT's step bound made it cycle between two poses to its iteration limit, a WARN that makes autonomous unavailable; fixed in the Autoware overlay | low | fixed |
| [003](003-world-reload-memory.md) | The Isaac world server grew ~130-150 MB per scenario: each ego's RTX lidar render product was never given back; the lidar is now kept per ego slot | low | fixed |
