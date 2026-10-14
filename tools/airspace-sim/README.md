# Airspace Replay

A small Rust-first, deterministic simulator/replay harness for the research contract in [AIRSPACE_COORDINATION_ARCHITECTURE_V1.md](../../docs/aerocommons/AIRSPACE_COORDINATION_ARCHITECTURE_V1.md).

## What it does

- Replays versioned, synthetic flight-intent fixtures.
- Uses explicit millimetre / millimetre-per-second integer units and a fixed simulation window.
- Refuses to compare coordinates across unqualified clocks, incompatible reference frames, or vertical datums.
- Rejects expired intents and out-of-bounds fixture inputs.
- Compares horizontal and vertical separation independently.
- Adds conservative relative-motion margins to the caller-supplied threshold so a potential encounter between sample instants is less likely to be missed.
- Requires an authority review for an emergency-priority scenario if no geometric conflict has already been found.
- Produces machine-readable JSON with expected-vs-actual classifications and evidence scope.

## Run

From the repository root:

```bash
cargo test --manifest-path tools/airspace-sim/Cargo.toml
cargo run --manifest-path tools/airspace-sim/Cargo.toml -- --verify
```

Or pass a specific scenario-set JSON path after `--verify`.

## Classification semantics

- `accept_for_planning_only`: no conflict was found within the synthetic model and declared scope.
- `conflict`: at least one pair meets the conservative synthetic conflict predicate.
- `degraded`: reserved for future explicit degraded-service profiles; not currently emitted by the basic evaluator.
- `reject`: malformed, expired, or out-of-bounds fixture inputs.
- `unresolved`: missing time qualification or incompatible coordinate frame / vertical datum.
- `requires_authority_review`: an emergency-priority intent remains subject to competent operational review.

## Important limitations

This is **not** an aircraft simulator, flight dynamics model, UTM/ATM service, collision-avoidance system, regulator, or authorization service. It does not model turn/climb performance, acceleration, wind, terrain, weather, sensor error distributions, non-cooperative traffic, or actual certified operating procedures.

The trajectories are straight and constant-velocity. The conservative margin is based on relative velocity and the maximum sampling interval; it is useful for testing the contract but is not a formally validated reachability analysis. The coordinate frame and separation thresholds are synthetic fixture values. They are not official separation minima and must never be reused for live operations.

A passing replay means only that the implementation matched these fixture expectations. It does not establish safety, airworthiness, certification, or permission to fly. Production evolution would require qualified aerospace expertise, independent verification, validated dynamics and uncertainty models, and competent-authority engagement.
