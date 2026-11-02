# Local benchmark sample

This is a measurement of the current CPU prototype, not a portable performance guarantee.

## Workload

- Scene: `swarm`; 500 seeded dynamic spheres plus one static floor (501 bodies total).
- Seed: `42`; fixed step: `1/120 s`; 900 steps per run (7.5 simulated seconds).
- Command: `quantumbody --scene swarm --count 500 --seed 42 --steps 900`.
- Runs: three consecutive Release runs; no warm-up samples were discarded.
- Timings cover `World::step()` and use `std::chrono::steady_clock`. The CLI reports per-step percentiles and totals.
- Hardware: Intel Core i3-13100F; Windows x64; MSVC 19.51 (`14.51.36231`), Release.

## Results

| Run | Wall time (s) | Mean step (ms) | P50 (ms) | P95 (ms) | P99 (ms) | Broad-phase candidates / contacts |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.414 | 0.460 | 0.448 | 0.766 | 1.282 | 350,860 / 350,860 |
| 2 | 0.428 | 0.476 | 0.465 | 0.723 | 1.480 | 350,860 / 350,860 |
| 3 | 0.361 | 0.401 | 0.423 | 0.592 | 0.865 | 350,860 / 350,860 |

The median of the three run-level P95 values is **0.723 ms/step**. The run-to-run spread reflects ordinary local timing variation. This sample does not establish scaling for other body counts, scene densities, compilers, hardware, or a real-time frame budget. The current step allocates temporary broad-phase and contact vectors, so it does not claim allocation-free simulation.
