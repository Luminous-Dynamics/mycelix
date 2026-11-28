# Agentic finite-game qualification harness

This standalone Cargo workspace compiles the exact production source files:

- `mycelix-workspace/sdk/src/agentic/game_theory.rs`
- `mycelix-workspace/sdk/src/agentic/game_theory_validation.rs`
- `mycelix-workspace/sdk/src/agentic/game_theory_verified.rs`

It uses `include!` to avoid copying production logic and has only the serde derive dependency. It is intentionally isolated from the rest of the SDK and Holochain dependency graph so unrelated workspace failures cannot mask this qualification slice.

## Local commands

From this directory:

```bash
cargo fmt --all -- --check
cargo generate-lockfile
cargo test --locked --all-targets
cargo clippy --locked --all-targets -- -D warnings
```

## Evidence boundary

The harness tests structural validation and pure-strategy Nash analysis for finite normal-form games. It does not qualify the legacy mechanism heuristic API, mixed or Bayesian equilibrium, repeated/extensive-form games, real human behaviour, fairness, legitimacy, or legal enforceability.

The dedicated workflow generates and uploads its resolved Cargo.lock because this environment cannot generate it locally. Until the lock is committed or independently preserved, dependency resolution is recorded per workflow run rather than claimed fully source-pinned.

## Independent comparator

See [COMPARATOR_PROTOCOL.md](COMPARATOR_PROTOCOL.md) for the exact Gambit/OpenSpiel cross-check contract, expected canonical equilibrium sets, metamorphic controls, and the required `MATCH` / `MISMATCH` / `UNSUPPORTED` / `NOT_RUN` result receipt. No external comparator run is claimed yet.
