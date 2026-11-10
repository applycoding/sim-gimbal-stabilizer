# Fixtures and golden traces

## RNG (xorshift32)

- Implementation: `cpp/src/sim/rng.cpp`
- Seed `0` is remapped to `1` before use.
- Uniform open interval: `(next_u32() + 1) / 2^32`
- Gaussian: Box–Muller with uniform draws in fixed call order (one `uniform01` for u1, one for u2 per `gaussian()`).

## Sensor noise draw order per `sample_sensors` call

When `perfect_sensors` is false, draws occur in this order:

1. gyro x, y, z Gaussian
2. accel x, y, z Gaussian
3. mag x, y, z Gaussian (if enabled)
4. encoder pitch uniform, encoder yaw uniform

## Golden comparison

- Host double traces: max abs diff `≤ 1e-9` rad between runs with the same seed and build.
- WASM float traces use `≤ 1e-5` rad when compared to native; do not compare at `1e-9`.
