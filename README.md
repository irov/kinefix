# kinefix

`kinefix` is a small deterministic math and collision library written in ISO C11.

It provides signed Q16.16 arithmetic, fixed-point vector/trigonometric operations,
PCG32 random streams, and deterministic collision queries. Authoritative functions
do not call the system floating-point math library.

Angles can be stored as binary turns: `kf_angle16_t` maps one full revolution to
the complete `uint16_t` range, so yaw wraps naturally. `kf_sangle16_t` represents
signed angular deltas and clamped pitch. Sine and cosine use a 4 KiB Q16.16
quarter-wave lookup table with deterministic integer interpolation; cosine is a
quarter-turn phase shift. Products and accumulated vector terms remain signed
Q32.32 until the outer Q16.16 result boundary. Fused multiply-add, projection,
hit-position, and sweep formulas do not narrow their intermediate terms.
Quadratic sweep coefficients remain Q32.32, while their Q64.64 discriminant uses
an unsigned 128-bit backend with a portable two-limb implementation for MSVC.
Public fixed values remain 32-bit.

`kf_fixed_from_float` is a convenience conversion for tests, tools, and other
non-authoritative boundaries. It truncates toward zero. Authoritative constants
should be stored as raw Q16.16 values or generated from decimal source data at
content-build time.

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

The pure C API is declared in `include/kinefix/kinefix.h`. C++ consumers include
`include/kinefix/kinefix.hpp`, which provides the `extern "C"` linkage wrapper.
