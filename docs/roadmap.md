# Roadmap

This roadmap narrows the original QuantumBody/SILNIK3D proposal into independently reviewable milestones. It is not a claim that the later items have shipped.

| Milestone | Scope | State |
| --- | --- | --- |
| M0 | CMake project, library boundary, docs and CI | Done |
| M1 | Fixed-step integration, sphere/box bodies, built-in scenes | Prototype implemented |
| M2 | Sweep-and-prune and primitive/OBB narrow phase | Prototype implemented; single-contact approximation |
| M3 | Frictional sequential impulses and angular response | Prototype implemented; no joints yet |
| M4 | Reproducibility checks, CSV/percentile metrics, failure scene | Basic version implemented; no thread parallelism |
| M5 | XPBD cloth and collision coupling | Planned |
| M6 | SPH fluids with a neighbor grid | Planned |
| M7 | CCD and improved contact manifolds | Planned |
| M8 | Repeatable benchmark harness and profiler visualization | CLI CSV and per-step percentiles exist; a three-run local sample is documented, but there is no controlled cross-machine harness or performance gate; visualization planned |
| M9 | Optional CUDA/Vulkan compute backend | Planned; requires suitable compiler/toolkit and device |
| M10 | Property/fuzz tests and sanitizer CI | Planned beyond the small current checks |
| M11 | Real-time renderer and interactive debug controls | Replay viewer exists; live renderer planned |
| M12 | Recorded demo and measured release results | Planned after implementation and hardware measurements |

## Release gates

Before promoting the project beyond prototype status: improve box contact manifolds, add analytic collision and conservation checks, run sanitizers, record build/compiler/hardware metadata with benchmark results, and verify CPU/GPU parity for any future compute path.
