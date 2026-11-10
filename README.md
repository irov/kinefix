# kinefix

Kinefix 1.4 is a standalone ISO C11 fixed-point geometry and physics library.
It has no dependency on Mengine, Python, graphics, networking, or a system
floating-point math library. The authoritative path uses integer arithmetic.

Include `kinefix/kinefix.h` from C for the complete public API: math, collision
queries, checked geometry, worlds, characters, bodies, constraints and snapshots.
C++ consumers include `kinefix/kinefix.hpp`, which provides the `extern "C"`
linkage wrapper.

The default world step is **25 Hz**, exactly `1/25` second (40 ms). Integer world
ticks and rational integration remainders are part of saved state. Frequencies
remain configurable. Equal initial states, stable application IDs, and ordered
commands produce the same results regardless of collider insertion or index
traversal order. The library does not own game commands or their scheduling.

Implemented components:

- Accurate sphere/capsule sweeps at rounded box edges, surface contacts and
  bounded depenetration; full-vector character motion and sequential sliding.
- A world with static, kinematic and dynamic objects, compound shapes,
  generation-checked handles, filters, exclusions and nonblocking triggers that
  report when shapes enter and leave them.
- Allocation-free BVH queries: first/all hits, contacts/overlaps, region objects,
  and batches. Buffers report the full required count on capacity errors.
- AABB, sphere, capsule, box, convex prism and double-sided static triangles in
  any orientation, including ramps and holes made by absent triangles, with
  clipped contact manifolds of up to four points.
- A capsule controller with slopes, checked steps, limited ground snap, support
  translation/rotation, support loss, a pit guard, landing speed, and actual
  movement/contact reporting.
- Rigid body dynamics with full 3D rotation: inertia tensors, contact torque,
  rolling and toppling, friction, restitution with a speed threshold, damping,
  per-body gravity scale and rotation locks, island sleep, kinematic targets,
  live body changes and velocity steering, and rigid or spring distance/rope
  links for chains.
- A substepped soft-contact solver whose impulses carry into the next step and
  into snapshots; resting stacks settle and sleep. Per-step contact reports
  mark pairs that began touching.
- Continuous collision of fast bodies including rotation, multiple bounces per
  tick, relative-motion queries, caller-controlled continuation, bounded
  quadratic trajectory approximation, and complete world snapshots.

Read [world/query contracts](docs/world.md), the [shape matrix and geometry
limits](docs/geometry.md), [character behavior](docs/character.md),
[body/constraint integration](docs/dynamics.md), and how the [Dante
prototype's physics maps onto the library](docs/dante.md). The public header
documents field meanings.

## Build and check

```sh
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -DKINEFIX_BUILD_TESTS=ON
cmake --build build-debug
ctest --test-dir build-debug --output-on-failure

cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DKINEFIX_BUILD_TESTS=ON -DKINEFIX_BUILD_BENCHMARKS=ON
cmake --build build-release
ctest --test-dir build-release --output-on-failure
./build-release/kinefix_benchmark 25 0
./build-release/kinefix_benchmark 25 1
./build-release/kinefix_level_benchmark 300
./build-release/kinefix_level_benchmark 300 60 4

cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DKINEFIX_BUILD_TESTS=ON -DKINEFIX_SANITIZERS=ON
cmake --build build-sanitize
ctest --test-dir build-sanitize --output-on-failure

cmake -S . -B build-portable -DCMAKE_BUILD_TYPE=Release -DKINEFIX_BUILD_TESTS=ON -DKINEFIX_PORTABLE_WIDE=ON
cmake --build build-portable
ctest --test-dir build-portable --output-on-failure
```

Warnings are errors on library, test and benchmark targets. Installation is
optional (`KINEFIX_INSTALL=ON`). Reservation and allocation failure are checked;
all resources are released by `kf_world_destroy`.

## Math and geometry

World queries return `kf_sweep_hit_t`, with the moving pose position and the
obstacle surface contact point. Characters use `kf_character_move` and keep
the complete `kf_character_state_t` between moves and in saved state.
The former collider-array queries and `kf_character_step` API have been removed;
callers must migrate to world queries and `kf_character_move`. Scalar/vector
math and random-number APIs remain available.

Signed Q16.16 math, vectors, fused multiply-add, PCG32 streams and binary-turn
angles remain available. `kf_angle16_t` maps a revolution to all 65536 values;
quarter-wave sine lookup and integer interpolation avoid platform math functions.
Wide products remain Q32.32 until their output boundary; quadratic discriminants
use either native unsigned 128-bit or the portable two-limb backend. MSVC, which
has no 128-bit integer type, uses the two-limb backend with `_umul128` and
`_udiv128` on x64 and `__umulh` on ARM64; every backend gives identical results.
Setting `KINEFIX_PORTABLE_WIDE=ON` forces the plain two-limb code even on
compilers with native 128-bit integers or intrinsics.

Scalar/vector math keeps assertion-based programmer contracts. Float/double conversions
are for tools and nonauthoritative input boundaries; they truncate toward zero,
assert on nonfinite/out-of-range input in Debug, and return zero in Release.
The checked physics API validates external data and distinguishes invalid input,
range/overflow, capacity, nonconvergence and no-hit results.

AI, weapon rules, damage/health, perks, explosions, destruction, waves, rewards,
navigation, network formats and persistence files belong to game code. Animation,
decals, visual body pieces and effects belong to the presentation layer. Import
Dante content with an explicit unit scale (see [docs/dante.md](docs/dante.md)):
the web prototype's pixel-like units are not directly interchangeable with the
library's documented world limits.
