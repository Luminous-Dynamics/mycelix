# Mycelix Holochain Rust toolchain qualification v0.2

**Status:** stacked on Governance path repair PR #6087. The nested Mycelix workspace's declared development pin remains Rust/Cargo 1.96.0. Rust/Cargo 1.99.1 is a candidate; no default toolchain promotion is proposed.

## Source prerequisite

The previous qualification run reached both Finance and Governance metadata, then Governance failed to load `mycelix-workspace/crates/mycelix-core-types/Cargo.toml`. The maintained crate lives at `mycelix-workspace/mycelix-core/libs/mycelix-core-types`.

PR #6087 corrects both Governance shared dependency paths to their maintained packages. This toolchain qualification is based on that branch, so compiler results are not conflated with a missing package path.

## Toolchain selection invariant

The nested workspace has its own `rust-toolchain.toml` pinned at 1.96.0. Merely installing 1.99.1 through an action does not make normal `cargo` calls use that candidate; rustup reports the root toolchain file as an override.

Every command therefore uses explicit selectors:
- `cargo +1.96.0` / `rustc +1.96.0` in the baseline job;
- `cargo +1.99.1` / `rustc +1.99.1` in the candidate job.

The WASM target is installed and inspected for the corresponding exact toolchain. This prevents a false candidate pass caused by the repository pin silently overriding CI's toolchain installation.

## Scope

This first qualification covers the Finance and Governance Holochain workspace roots:
- `mycelix-workspace/mycelix-finance`;
- `mycelix-workspace/mycelix-governance`.

Both versions independently run:
1. Exact rustc/Cargo identity checks.
2. `wasm32-unknown-unknown` target availability.
3. Locked metadata for both cluster workspaces.
4. Full locked Governance dependency resolution.
5. Finance and Governance `cargo check --workspace`.
6. Existing targeted Finance author-binding security regression suites.
7. Governance workspace library tests.
8. Release WASM builds for the two clusters.
9. Lockfile immutability.

## Why this is scoped

The nested workspace contains many separately migrated and incomplete sub-workspaces. Finance has a known stale `zomes/shared` test module; its source API mismatch is why the existing verified gate does not use `--all-targets`. The workflow mirrors the existing verified surface and keeps those known-red test targets out of the toolchain comparison rather than hiding or rewriting them.

A green result qualifies only these cluster roots, named packages and commands. It is not evidence for every Mycelix cluster, every Holochain target or a live conductor.

## Invariants

- No change to `mycelix-workspace/rust-toolchain.toml`.
- No Holochain `hdk`, `hdi`, zome-type or Cargo dependency version changes.
- No Cargo.lock changes.
- No edition/MSRV changes.
- No Nix flake/Holonix input changes.
- No DNA packaging, conductor, network or production deployment.
- No Finance/Governance domain logic changes in the toolchain PR itself; the path repair is isolated in PR #6087.

## Benchmark and SOTA programme

A newer compiler is not automatically a better Holochain system. Compare:
- native compile/test wall time and peak RSS;
- release WASM compile time and peak RSS;
- WASM output size per zome;
- repeated builds and output hashes;
- actual conductor-level request latency/throughput for identical DNA artifacts;
- transaction validation and signature verification throughput;
- proof verification costs where ZKP features are enabled.

Use rustc-perf for compiler-wide baseline comparisons, Cargo `--timings` for build-graph critical paths, and Mycelix-specific workload tests for application behavior. Keep all Holochain version, DNA, conductor, storage, network and workload inputs identical. Report compiler build performance separately from end-to-end Holochain runtime.

## Statistical standards

- 5+ paired runs for exploratory compilation/size comparisons.
- 30+ independent runs for ordinary latency claims.
- 10,000+ observations per condition for p99 claims, or a justified alternative design.
- Rotate compiler order to reduce thermal/order bias.
- Record exact toolchain commit, lockfile digest, runner image, kernel, target and raw logs.
- Report failed/skipped trials separately.

No compiler or application performance advantage is claimed until matched data exist.

## Promotion sequence

1. Merge/qualify the Governance path repair prerequisite.
2. Obtain exact-head Rust 1.96.0 and 1.99.1 hosted results.
3. Resolve any lockfile, compile, test or WASM failures without removing `--locked`.
4. Inspect any candidate-only behavior or numerical differences.
5. Run matched native/WASM and actual conductor benchmarks.
6. Propose a separate default toolchain change only after evidence supports it.

## References

- Rust compiler performance testing: https://rustc-dev-guide.rust-lang.org/tests/perf.html
- Cargo timings: https://doc.rust-lang.org/cargo/reference/timings.html
- Holochain developer documentation: https://developer.holochain.org/
