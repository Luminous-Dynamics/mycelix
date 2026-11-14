# IAE → LISE owner adapter

Executable conformance adapter for the IAE v0.2 institutional-assurance packet and the standalone LISE exact-operation reference.

## Dependency and owner boundary

This crate uses a local path dependency on `../ipb-finality-reference`. It does not copy or reimplement LISE's finality state machine. It translates the IAE operation projection into LISE's typed `FinalityInput` and calls `evaluate_finality`.

The branch is stacked on the LISE reference branch. Merge order: first [LISE reference PR #6163](https://github.com/Luminous-Dynamics/mycelix/pull/6163), then this adapter PR, then IAE's vertical-slice PR #6169 can pin the accepted owner contract.

## Run

```sh
cargo fmt --manifest-path lise-benchmarks/iae-lise-owner-adapter/Cargo.toml --check
cargo check --manifest-path lise-benchmarks/iae-lise-owner-adapter/Cargo.toml --all-targets
cargo test --manifest-path lise-benchmarks/iae-lise-owner-adapter/Cargo.toml --all-targets
cargo clippy --manifest-path lise-benchmarks/iae-lise-owner-adapter/Cargo.toml --all-targets -- -D warnings
```

## Expected decisions

The same IAE packet is evaluated at three distinct points:
1. Original preflight before dispatch: `Permit`.
2. Original possible effect after timeout: `Hold(PriorOutcomeUnknown)`.
3. A potential reattempt after authority revocation: `Deny(AuthorityRevoked)`.

The second and third results are not contradictory: a new attempt is forbidden, while the outcome of the original attempt remains unresolved.

## Claim ceiling

The fixture is synthetic. Operation and constitution digests are labeled placeholders and are not cryptographically verified. Authority, approval and resource states remain owner-supplied synthetic facts. These tests prove adapter correspondence to the LISE reference inputs, not real legal authority, source authenticity or provider behavior.

The branch intentionally has no committed lockfile yet; it remains draft until the exact-head workflow resolves and the dependency graph can be locked and reviewed.
