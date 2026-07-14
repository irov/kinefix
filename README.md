# kinefix

`kinefix` is a small deterministic math and collision library written in ISO C11.

It provides signed Q32.32 arithmetic, fixed-point vector/trigonometric operations,
PCG32 random streams, and deterministic collision queries. Authoritative functions
do not call the system floating-point math library.

Angles can be stored as binary turns: `kf_angle16_t` maps one full revolution to
the complete `uint16_t` range, so yaw wraps naturally. `kf_sangle16_t` represents
signed angular deltas and clamped pitch. Sine and cosine use an 8 KiB Q32.32
quarter-wave lookup table with deterministic integer interpolation; cosine is a
quarter-turn phase shift. Radian conversion and the older Q32.32 trigonometry API
remain available for compatibility.

Collision primitives and queries include:

- AABB, sphere, ray, and Y-up capsule shapes;
- AABB/sphere/capsule overlap tests;
- parametric raycasts and continuous sphere/capsule sweeps;
- `kf_hit_t` results with fraction, distance, position, normal, stable object id,
  and initial-overlap state;
- deterministic world queries selecting the earliest hit and then the lowest
  object id when hit fractions are equal;
- a capsule-based character controller with continuous movement, skin width,
  stepping, sliding, floor/ceiling contacts, and high-speed tunneling prevention.

`kf_capsule_t.position` is the bottom-center reference point, `height` is its full
height, and the capsule is aligned with the Y axis. Sweep displacement is the full
motion for one query; returned `fraction` is always in the inclusive `[0, 1]`
interval. Capsule-vs-AABB sweeps conservatively cast against the AABB expanded by
the capsule radius, so they prefer an early contact over tunneling at box corners.

```sh
cmake -S . -B build -DKINEFIX_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The public API is declared in `include/kinefix/kinefix.h` and is usable from both C
and C++ through `extern "C"` guards.
