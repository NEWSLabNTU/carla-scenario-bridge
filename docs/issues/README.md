# Issues

One file per defect: what is wrong, why it matters, how it was confirmed, and the fix or
what is still open.

| # | Issue | Severity | Status |
|---|---|---|---|
| [001](001-soil-tile-rebuild-memory.md) | Each soil tile rebuild left cooked collision in the Isaac world server (~0.4 MB, then 64 KB); now no mesh is cooked after load | low | fixed |
| [002](002-mid-sand-mrm.md) | Emergency stops on town01_soil: one NDT scan at its iteration limit makes autonomous unavailable | low | open |
| [003](003-world-reload-memory.md) | The Isaac world server grows ~130-150 MB with every scenario's world load | low | open |
