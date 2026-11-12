# Mycelix Holochain Rust toolchain qualification v0.1

**Status:** qualification proposal. The nested Mycelix workspace's declared development pin remains Rust/Cargo 1.96.0. Rust/Cargo 1.99.1 is a candidate; this change does not promote it as the new default.

## Scope and rationale

The central toolchain contract is `mycelix-workspace/rust-toolchain.toml`, currently Rust 1.96.0 with rustfmt, Clippy and `wasm32-unknown-unknown`.

This first qualification covers the Finance and Governance Holochain workspace roots already identified as verified green in the nested workspace's qualification notes:

- `mycelix-workspace/mycelix-finance`
- `mycelix-workspace/mycelix-governance`

It compares the exact 1.96.0 baseline with exact 1.99.1 using the same source revision and independent lockfiles.

## Why the gate is deliberately scoped

The nested workspace contains many partially migrated and independently rooted clusters. Root-level `cargo check --workspace` does not cover all of them. The Finance cluster also has a known stale `zomes/shared` test module with missing APIs; adding `--all-targets` would compile that stale test code and create a permanently red signal.

Accordingly, this qualification mirrors the existing verified test surface:
- `cargo check --workspace` for Finance and Governance;
- targeted Finance author-binding security regressions;
- Governance workspace library tests;
- release WASM builds for Finance and Governance;
- locked metadata and lockfile immutability.

It intentionally does not run `--all-targets` or all tests across every cluster. That is a scoped evidence boundary, not a waiver for known defects.

## Important invariants

- Holochain `hdk`, `hdi`, and zome type dependencies remain exactly pinned in each cluster manifest.
- No dependency version, Cargo.lock, edition or MSRV changes.
- No default toolchain pin change.
- No Nix flake or Holonix input updates.
- No Holochain conductor, DNA or network deployment.
- No finance/governance domain logic or security policy changes.

## Industry / SOTA qualification standard

For a Holochain workspace, compile speed alone is not a sufficient quality metric. A compiler candidate must preserve:

1. Native integrity/coordinator crate compilation.
2. Security regression behavior (forged author-binding values remain rejected).
3. WASM zome compilation for the actual `wasm32-unknown-unknown` target.
4. Reproducible dependency resolution with unchanged Cargo.lock.
5. Runtime semantics under an actual conductor before production promotion.

The upstream compiler comparison should use rustc-perf for compiler-level measurements. Mycelix-specific performance measurements should separately capture:
- native check/test wall time and peak RSS;
- release WASM compile wall time and peak RSS;
- WASM output size by zome;
- deterministic artifact hashes for same-toolchain independent builds;
- zome execution latency and throughput under an actual Holochain conductor;
- transaction validation and signature-verification throughput;
- proof verification costs where ZKP features are enabled.

The compiler upgrade itself should not be credited for end-to-end Holochain performance without matched conductor, DNA, storage, network, and workload inputs.

## Statistical targets

- 5+ paired independent runs for exploratory compile/size comparisons.
- 30+ runs for ordinary latency-distribution claims.
- At least 10,000 request observations per condition, or a justified statistical design, before p99 claims.
- Rotate compiler order to reduce thermal and runner-order bias.
- Retain exact compiler commit, Cargo identity, lockfile SHA-256, runner image, kernel, target triple, and raw logs.
- Report native and WASM results separately.

## Promotion sequence

1. Obtain exact-head 1.96.0 and 1.99.1 hosted results.
2. Resolve any candidate-only compile, test, Clippy or WASM failure.
3. Verify selected security regression tests still pass.
4. Run paired build/size measurements.
5. Run actual conductor-level runtime comparisons on matched zome artifacts.
6. Only then propose a separate Rust pin change for the nested workspace flake/devShell contract.

A green focused lane qualifies only the two named clusters and commands. It is not a claim that every Mycelix sub-workspace is ready for Rust 1.99.1.

## References

- Rust compiler performance testing: https://rustc-dev-guide.rust-lang.org/tests/perf.html
- Cargo timings: https://doc.rust-lang.org/cargo/reference/timings.html
- Holochain documentation: https://developer.holochain.org/
