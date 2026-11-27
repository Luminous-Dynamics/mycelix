# Rust toolchain policy

**Policy snapshot:** 2026-11-26  
**Current stable baseline:** Rust 1.100.0 (released 2026-11-12)

## Keep three toolchain concepts separate

| Concept | Meaning | Policy |
|---|---|---|
| Current stable | Preferred compiler for active development and current CI | Rust 1.100.0 as of this snapshot |
| MSRV | Oldest compiler a particular crate promises to support | Declare per crate with `package.rust-version`; do not raise it merely because stable advances |
| Historical qualification toolchain | Exact compiler used to produce a qualification receipt | Preserve its version and commit identity; requalify on a newer toolchain with a new receipt |

The root repository does not define one Cargo workspace or one universal MSRV. It contains multiple independently managed Rust workspaces. Their MSRV promises must therefore be audited from their individual manifests, not inferred from this policy.

## Active CI policy

The main multi-cluster workflow uses `dtolnay/rust-toolchain@stable`, which tracks current stable. Focused workflows that issue machine-readable qualification results must pin the exact toolchain version and record its identity.

As of this snapshot, the Terra Preta focused workflows use Rust 1.100.0:

- Robust hydraulic experiment design — PR [#6664](https://github.com/Luminous-Dynamics/mycelix/pull/6664)
- Cost-matched uniform portfolio comparison — PR [#6708](https://github.com/Luminous-Dynamics/mycelix/pull/6708)
- Saturated lab-core observation operators — PR [#6666](https://github.com/Luminous-Dynamics/mycelix/pull/6666)
- Observation/provenance contract — PR [#6659](https://github.com/Luminous-Dynamics/mycelix/pull/6659)

The Food Lot reconciliation harness is being moved to the same current-stable baseline by the separate PR for [issue #6721](https://github.com/Luminous-Dynamics/mycelix/issues/6721).

For a focused qualification workflow:

1. Pin `toolchain: 1.100.0` (not a moving `stable` alias).
2. Assert `rustc --version` reports 1.100.0.
3. Retain `cargo fmt --check`, tests, and strict Clippy.
4. Record `rustc --version --verbose`, the exact source commit, workflow run ID/attempt, and relevant input digests.
5. Apply formatter changes reported by the pinned formatter; never weaken the formatting gate to preserve an older layout.

## Historical evidence must not be silently rewritten

The Finance Exact-Head and D6U Holochain runtime qualification workflows currently pin Rust 1.96.1 and assert its compiler commit hash. Those pins form part of existing qualification identity. They must not be changed in place and then represented as if the old evidence were produced with Rust 1.100.0.

A newer qualification should instead:
- run as a separately identified 1.100.0 qualification;
- record the full new compiler identity and exact source head;
- rerun formatting, tests, Clippy and domain-specific evidence verifiers;
- preserve the older 1.96.1 run and artifacts as historical evidence.

Archived D6U workflow snapshots are immutable reference material and are not toolchain-update targets.

## Upgrade cadence

Review current stable when a Rust release is announced. Update development and focused qualification baselines after checking:
- compiler and Cargo compatibility of the relevant workspace;
- formatter output;
- strict Clippy changes;
- test and runtime behavior;
- the distinction between new compiler coverage and declared MSRV.

Do not update `rust-version` without an explicit compatibility decision and MSRV test coverage. Current stable and MSRV are different promises.

## Security note

Rust 1.100.0 stabilizes Cargo's minimum-publish-age dependency-resolution control. A repository-wide dependency cooldown should be a separate supply-chain policy change: first verify registry support, lockfile reproducibility, and exceptions for urgent security updates. It is not enabled automatically by selecting Rust 1.100.0.

## Claim boundary

A passing Rust 1.100.0 build proves compilation and the executed checks on that toolchain and source head. It does not automatically prove compatibility with every older MSRV, hardware target, Holochain runtime, or deployment environment.
