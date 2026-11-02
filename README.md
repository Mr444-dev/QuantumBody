# QuantumBody

**A small, dependency-free 3D rigid-body physics sandbox in modern C++.** QuantumBody is an educational engine prototype built around a fixed-step simulation loop, oriented-box and sphere collision detection, a sequential-impulse contact solver, reproducible scenes, and a lightweight replay viewer.

> **Status:** prototype / milestone M1–M4 foundation. The CPU rigid-body path, CLI scenes, seeded replay, per-step timing output, and a small verification suite are implemented. CUDA, fluids, cloth, joints, continuous collision detection, and a full real-time renderer remain future work. A three-run local timing sample is recorded in [docs/benchmark.md](docs/benchmark.md); it is a reproducible command run, not a controlled cross-machine performance claim.

## What works today

- C++20 physics library with no third-party runtime dependencies.
- Fixed-step semi-implicit Euler integration for linear motion and normalized quaternion orientation.
- Sphere and oriented-box shapes; sphere-sphere, sphere-OBB, and OBB-OBB narrow-phase checks (15-axis SAT).
- Sweep-and-prune broad phase using conservative world AABBs.
- Iterative sequential impulses for normal response and Coulomb friction, including angular impulse response and penetration bias.
- Deterministic built-in `stack`, `swarm`, and `drop` scenes, plus a `failure-lab` stress scene.
- JSON trajectory export, CSV per-step metrics, and an offline isometric replay viewer.
- CTest checks for free-fall behavior, repeatability, floor contacts, and invalid time steps.
- GitHub Actions build and test workflow on Windows and Ubuntu.

## Build

Requirements: CMake 3.21+, and a C++20 compiler (MSVC 2022, GCC 11+, or Clang 14+).

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

On a single-configuration generator such as Ninja, `-C Release` is harmless for CTest; configure with `-DCMAKE_BUILD_TYPE=Release` as shown.

## Run a scene

```powershell
.\build\Release\quantumbody.exe --scene stack --steps 600
.\build\Release\quantumbody.exe --scene swarm --count 500 --seed 42 --steps 900
.\build\Release\quantumbody.exe --scene drop --csv work\drop.csv
```

The program prints one JSON metrics record to stdout. `--csv FILE` writes step time, broad-phase candidate count, and contact count per step. Example scene names and all flags are listed by `quantumbody --help`.

## Record and inspect a replay

```powershell
.\build\Release\quantumbody.exe --scene stack --steps 600 --record-every 3 --trajectory web\trajectory.json
python -m http.server 8000
```

Open `http://localhost:8000/web/` and load `trajectory.json` with the file picker. The viewer also accepts `?file=trajectory.json` when the replay sits next to `index.html`.

## Example output

The JSON summary includes the seed, body count, simulated duration, measured wall duration, steps/second, p50/p95/p99 step time, and aggregate broad-phase/contact counts. Times depend on compiler, build mode, and hardware. Run multiple trials before drawing performance conclusions.

## Repository map

```text
include/quantumbody/   Public math, body, world, and scene API
src/                  Solver, collision pipeline, scenes, and CLI
tests/                Small deterministic verification suite
web/                  Offline replay viewer (no JS dependencies)
docs/                 Architecture, algorithms, scope, and roadmap
.github/workflows/    Windows and Ubuntu CI
```

## Physics model and current limits

See [docs/architecture.md](docs/architecture.md) for the update pipeline, [docs/physics.md](docs/physics.md) for formulas and solver conventions, and [docs/benchmark.md](docs/benchmark.md) for one measured local run. Contacts currently use one support-point midpoint per body pair. This is compact and useful for a prototype, but less stable than a clipped multi-point manifold for wide box stacks. Sphere centers fully inside boxes use a simplified penetration fallback. The broad phase is a one-axis sweep and can degrade in highly overlapping scenes. Shapes are limited to spheres and boxes. CCD, articulated joints, sleeping, SIMD, GPU compute, fluid/cloth solvers, and live rendering are not implemented yet.

## Roadmap

1. Improve contact manifolds and regression coverage; add SAT reference cases.
2. Add persistent contacts, distance/hinge constraints, and a chain scene.
3. Add XPBD cloth and a neighbor-grid SPH fluid scene.
4. Add an optional CUDA compute backend with CPU/GPU parity checks.
5. Replace replay-only visualization with an interactive real-time debug renderer and profiler.
6. Add CCD, stress/failure cases, sanitizer jobs, and recorded benchmark methodology.

## License

MIT. See [LICENSE](LICENSE).

---

Polish documentation: [README.pl.md](README.pl.md).
