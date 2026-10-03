# Built-in example scenes

Scenes are compiled generators in `src/scenes.cpp` so they remain deterministic and do not need a JSON parser.

| Scene | What it exercises | Example |
| --- | --- | --- |
| `stack` | Grid of 125 dynamic boxes dropping onto a static floor | `quantumbody --scene stack --steps 900 --trajectory web/trajectory.json` |
| `swarm` | Seeded sphere cloud and broad-phase scaling | `quantumbody --scene swarm --count 1000 --seed 42 --steps 600 --csv work/swarm.csv` |
| `drop` | Repeated sphere impacts with a high restitution | `quantumbody --scene drop --steps 1200 --trajectory web/trajectory.json` |
| `failure-lab` | Thin-wall tunneling and large-coordinate precision | `quantumbody --scene failure-lab --steps 180 --trajectory web/trajectory.json` |

On Windows use `build/Release/quantumbody.exe`; on single-configuration generators the executable is usually `build/quantumbody`. The failure lab intentionally demonstrates missing CCD and precision limits; its output is not a pass/fail robustness claim.
