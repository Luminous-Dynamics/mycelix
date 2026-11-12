# Mycelix Holochain Rust toolchain qualification v0.3

**Status:** stacked on Governance shared-path repair PR #6087. The nested Mycelix workspace's declared development pin remains Rust/Cargo 1.96.0. Rust/Cargo 1.99.1 and 1.100.0 are isolated candidates; no default toolchain promotion is proposed.

## Source prerequisite and discovered blockers

The first qualification run found Governance path errors: mycelix-core-types and feldman-dkg pointed into a nonexistent workspace-local crates directory. The maintained packages are under mycelix-workspace/mycelix-core/libs/.

PR #6087 corrects the two paths without changing package code, dependency versions, features or Holochain pins. This qualification branch is stacked on that repair.

A later run reached full locked dependency resolution for Finance on both Rust 1.96.0 and 1.99.1, then Cargo reported that Finance Cargo.lock would need to change under --locked. This is a pre-existing lock/manifest consistency problem, not yet attributed to either compiler. The default branch and checked-in lockfiles are not changed to get a green build.

## Toolchain selection invariant

The nested workspace has its own rust-toolchain.toml pinned at 1.96.0. Installing 1.99.1 through an action does not override the repository-local rustup selection for commands run inside the nested workspace.

Every command therefore uses explicit selectors:
- cargo +1.96.0 / rustc +1.96.0 for the baseline;
- cargo +1.99.1 / rustc +1.99.1 for the candidate.

The WASM target is installed and inspected for the matching compiler. This prevents a false candidate pass caused by the repository pin overriding the CI-installed compiler.

## Independent qualification matrix

The workflow now creates six independent jobs:

| Cluster | Rust 1.96.0 baseline | Rust 1.99.1 candidate | Rust 1.100.0 candidate |
|---|---|---|
| Finance | Rust 1.96.0 | Rust 1.99.1 | Rust 1.100.0 |
| Governance | Rust 1.96.0 | Rust 1.99.1 | Rust 1.100.0 |

One cluster's path or lockfile failure no longer skips the other cluster's compile, test and WASM checks.

Each cell performs:
1. Exact rustc/Cargo identity checks and target availability.
2. Locked workspace metadata (no dependency resolution).
3. Full locked dependency resolution.
4. cargo check --locked --workspace.
5. The existing targeted Finance author-binding security regression suites or Governance workspace library tests.
6. Release WASM compilation for the actual wasm32-unknown-unknown target.
7. Per-cluster Cargo.lock digest/immutability check.

The Finance WASM command excludes host-only finance-leptos-types, matching the existing build contract. Finance's known stale zomes/shared test module remains outside the established test surface; the targeted integrity regressions remain explicit.

## Failure diagnostics without mutating the checkout

When a locked resolution or build stage fails, the workflow extracts the exact Git source snapshot into a temporary runner directory and runs an unlocked diagnostic only there. It captures the generated lockfile and a diff against the checked-in lockfile, along with compiler identity and stderr.

This is not a qualification pass and does not change the checked-out workspace. The real qualification always retains --locked, and the final step verifies that the original cluster Cargo.lock digest remains byte-identical.

If the diff identifies legitimate missing lock entries, regenerate the checked-in lockfile with Cargo under the declared toolchain in a separate reviewed change. Never hand-edit Cargo.lock or set --locked aside in the qualification job.

## Scope and evidence boundary

The nested workspace contains many separately migrated and incomplete sub-workspaces. These two cluster roots are the first scope because existing CI already exercises selected Finance and Governance gates.

A passing cell qualifies only the named cluster, toolchain, target and commands. It is not evidence for every Mycelix cluster, every Holochain target, the full finance test directory, DNA packaging, a live conductor or production deployment.

## Invariants

- No change to mycelix-workspace/rust-toolchain.toml.
- No Holochain hdk, hdi, zome-type or Cargo dependency version changes.
- No Cargo.lock changes in the toolchain qualification PR.
- No edition/MSRV changes.
- No Nix flake/Holonix input changes.
- No DNA packaging, conductor, network or production deployment.
- No Finance/Governance domain logic changes in the toolchain PR; the shared-path repair is isolated in PR #6087.

## Benchmark and SOTA programme

Rust 1.100.0 was released on 12 November 2026. It adds stabilized Cargo lints, minimum publish-age controls and a changed internal build directory layout; the qualification therefore includes a third exact compiler version while keeping the repository default unchanged. The release's new Cargo features are evaluated separately from Holochain compiler compatibility.

A newer compiler is not automatically a better Holochain system. Compare:
- native compile/test wall time and peak RSS;
- release WASM compile time and peak RSS;
- WASM output size per zome;
- independent repeated build artifact hashes;
- actual conductor-level request latency/throughput for identical DNA artifacts;
- transaction validation and signature-verification throughput;
- proof verification costs where ZKP features are enabled.

Use rustc-perf for compiler-wide baseline comparisons, Cargo --timings for build-graph critical paths, and Mycelix-specific workload tests for application behavior. Keep Holochain version, DNA, conductor, storage, network and workload inputs identical. Report compiler performance separately from end-to-end Holochain runtime.

## Statistical standards

- 5+ paired runs for exploratory compilation/size comparisons.
- 30+ independent runs for ordinary latency claims.
- 10,000+ observations per condition for p99 claims, or a justified alternative design.
- Rotate compiler order to reduce thermal/order bias.
- Record exact toolchain commit, lockfile digest, runner image, kernel, target and raw logs.
- Report failed/skipped trials separately.

No compiler or application performance advantage is claimed until matched data exist.

## Promotion sequence

1. Qualify the Governance shared-path repair prerequisite.
2. Obtain independent exact-head Rust 1.96.0 and 1.99.1 results for Finance and Governance.
3. Diagnose Finance lockfile drift in the disposable snapshot; if legitimate, make a separate Cargo-generated lockfile repair.
4. Resolve any compile, test or WASM failure without removing --locked.
5. Inspect candidate-only behavior or numerical differences.
6. Run matched native/WASM and actual conductor benchmarks.
7. Propose a separate default toolchain change only after evidence supports it.

## References

- Rust compiler performance testing: https://rustc-dev-guide.rust-lang.org/tests/perf.html
- Cargo timings: https://doc.rust-lang.org/cargo/reference/timings.html
- Holochain developer documentation: https://developer.holochain.org/
