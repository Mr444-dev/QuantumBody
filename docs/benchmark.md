# Local benchmark sample

This is a local measurement of the current CPU prototype, not a portable performance guarantee.

## Workload and method

- Scene: `swarm`; 500 seeded dynamic spheres plus one static floor (501 bodies total).
- Seed: `42`; fixed step: `1/120 s`; 900 steps per process (7.5 simulated seconds).
- Command: `quantumbody --scene swarm --count 500 --seed 42 --steps 900`.
- Runs: eight consecutive MSVC Release process runs. No first-step or warm-up samples were discarded.
- Per-step timings use `std::chrono::steady_clock`; `Total step time` is the sum of all 900 step samples, not process startup time.
- Hardware: Intel Core i3-13100F; Windows x64; MSVC 19.51 (`14.51.36231`), Release.

## Results

| Run | Total step time (s) | Mean step (ms) | P50 (ms) | P95 (ms) | P99 (ms) | Broad-phase candidates / contacts |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.427 | 0.474 | 0.543 | 0.709 | 1.021 | 350,860 / 350,860 |
| 2 | 0.438 | 0.487 | 0.544 | 0.786 | 1.284 | 350,860 / 350,860 |
| 3 | 0.404 | 0.449 | 0.518 | 0.662 | 0.757 | 350,860 / 350,860 |
| 4 | 0.416 | 0.462 | 0.498 | 0.684 | 0.842 | 350,860 / 350,860 |
| 5 | 0.471 | 0.523 | 0.541 | 0.712 | 1.386 | 350,860 / 350,860 |
| 6 | 0.438 | 0.487 | 0.463 | 0.649 | 1.345 | 350,860 / 350,860 |
| 7 | 0.402 | 0.447 | 0.458 | 0.631 | 0.850 | 350,860 / 350,860 |
| 8 | 0.390 | 0.433 | 0.467 | 0.627 | 0.718 | 350,860 / 350,860 |

The median of the eight run-level P95 values is **0.673 ms/step**. The previous three-run sample recorded 0.723 ms/step, so this sample is 6.9% lower by that summary statistic. The samples were not paired or run in a controlled benchmark harness, and their spread overlaps; the difference is not evidence of a guaranteed speedup. The stronger result is structural: the CTest replacement-global-operator-new probe observed zero C++ `new`/`new[]` calls during 128 steady-state `World::step()` calls after the scratch buffers were warmed. That guarantee applies while the body count and pair/contact high-water marks stay within retained vector capacity. This sample does not establish scaling for other body counts, scene densities, compilers, hardware, or a real-time frame budget.
