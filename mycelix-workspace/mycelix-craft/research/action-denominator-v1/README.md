# Immutable authorized service-action denominator V1

Standalone Rust reference core for issue #6792. It models the authorized denominator as a canonical, immutable set of action records bound to one service episode, scope, policy profile and authority decision.

## Run

From this directory:

```sh
rustup toolchain install 1.96.0 --profile minimal
rustup component add rustfmt clippy --toolchain 1.96.0
cargo +1.96.0 generate-lockfile
cargo +1.96.0 fmt --all -- --check
cargo +1.96.0 test --locked
cargo +1.96.0 clippy --all-targets --locked -- -D warnings
```

## Contract

- The plan digest commits to the full canonical action set, not only a count.
- Action IDs, prerequisite graph and evidence profiles are validated before sealing.
- Observations bind to the exact plan digest, episode, zone, jurisdiction, context and action ID.
- Duplicate observation IDs are idempotent only for identical canonical content; changed content under one ID is Conflict.
- Multiple observations from one independence group count once.
- Explicit expiry/revocation invalidates the matching evidence group; a dispute produces Conflict. A stale old observation is excluded but does not silently revoke a newer current observation.
- Prerequisites must be covered before a dependent action can count.
- A denominator amendment requires a new sequential revision, exact supersedes digest and a new authority decision digest. Old plans remain independently replayable.
- Reports carry numerator, denominator, basis points, exact covered/uncovered action IDs and rejected-observation reasons.

## Current test suite

Canonical ordering and SHA-256 known answer; 6-authorized/5-observed arithmetic; duplicate idempotence and conflicting identity; common-cause de-duplication; wrong-zone rejection; expired/revoked/disputed evidence; prerequisite gating; authority-bound amendment/historical replay; same-count/different-set digest distinction; cyclic and duplicate action IDs; and plan lifetime abstention.

## Benchmark protocol

See [BENCHMARK_PROTOCOL_V1.md](BENCHMARK_PROTOCOL_V1.md) for matched baselines, frozen correctness matrix, target qualification rates, workload scaling and performance methodology.

## Claim ceiling

A green run establishes deterministic accounting over caller-supplied, typed plans and observations. It does not authenticate the issuing authority, observation sources, permit/qualification revocation outside the plan packet, field completion, professional approval, physical water restoration or permission to close an incident. The upstream owner must actually ratify the authority decision; this crate checks binding fields and revision transitions, not the legal or organizational truth of that authority.
