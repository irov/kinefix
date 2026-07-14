# kinefix

`kinefix` is a small deterministic math and collision library written in ISO C11.

It provides signed Q32.32 arithmetic, fixed-point vector/trigonometric operations,
PCG32 random streams, AABB/sphere sweeps, line queries, and a deterministic
non-rotating character-body step. Authoritative functions do not call the system
floating-point math library.

```sh
cmake -S . -B build -DKINEFIX_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The public API is declared in `include/kinefix/kinefix.h` and is usable from both C
and C++ through `extern "C"` guards.
