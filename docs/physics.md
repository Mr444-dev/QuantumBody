# Physics notes

## State and integration

Each dynamic body stores position `x`, orientation quaternion `q`, linear velocity `v`, angular velocity `ω`, accumulated force `F`, and accumulated torque `τ`. The fixed-step update uses semi-implicit Euler:

```text
v(t + h) = v(t) + h (g + F / m)
x(t + h) = x(t) + h v(t + h)
ω(t + h) = ω(t) + h I_world^-1 τ
q(t + h) = normalize(q(t) + h/2 [0, ω] q(t))
```

The inertia tensor is diagonal in body space for spheres and boxes. The current implementation rotates a vector into body space, applies the diagonal inverse inertia, and rotates it back. Gyroscopic torque is omitted.

## Broad and narrow phase

An oriented box's conservative world AABB has half-size

```text
e_world = |R ex| hx + |R ey| hy + |R ez| hz
```

where `R ex` etc. are its rotated local axes. Proxies are sorted by AABB minimum x; pairs are considered until x intervals separate, then checked on y and z.

Sphere pairs use center distance. Sphere/box pairs transform the sphere center to box-local coordinates and clamp it to the box. Box pairs test the 3 axes of each box and all 9 pairwise cross products. A separating axis rejects the pair; the minimum-overlap axis gives the contact normal. The current contact point uses the midpoint of the support planes and anchors tangentially at the dynamic body (or the smaller body for a dynamic pair). It is an approximation rather than a clipped face manifold.

## Contact impulse

For a unit direction `n`, contact offsets `rA`, `rB`, and inverse mass/inertia, the scalar effective inverse mass is

```text
k(n) = invMassA + invMassB
     + n · ((I_A^-1 (rA × n)) × rA + (I_B^-1 (rB × n)) × rB)
```

Relative contact velocity is `vB + ωB × rB - vA - ωA × rA`. The solver computes a normal impulse, accumulates it with a zero lower bound, and applies equal/opposite impulses. Restitution is enabled only when the pre-solve closing speed is below 1 m/s. A small penetration slop and velocity bias reduce visible drift. Friction impulses are accumulated and clamped to `μ * normalImpulse`, with `μ = sqrt(μA μB)`.

## Known numerical and physical limits

- No continuous collision detection: a sufficiently fast body can tunnel through thin shapes. `failure-lab` exposes this limitation.
- One approximate contact per overlapping pair: box stacks may wobble or sink under load.
- For a sphere whose center is inside an OBB, the fallback selects a nearby face direction but currently reports only the sphere radius as penetration. Deep containment can therefore remain under-corrected; this path has no dedicated regression test.
- No warm starting, sleeping, joints, gyroscopic term, or mass-ratio conditioning.
- Position correction uses a contact velocity bias, not a separate split-impulse position pass.
- The broad phase is single-threaded and may approach quadratic work when many AABBs overlap.
- Floating-point results are deterministic for a fixed execution environment, not guaranteed bitwise-identical across CPU/compiler combinations.
