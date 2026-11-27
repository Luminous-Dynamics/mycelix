# Immutable authorized service-action denominator V1

This is a standalone, dependency-light Rust reference core for issue #6792. It treats the authorized action denominator as a canonical, immutable set bound to an episode, scope, policy profile and authority decision. It is deliberately separate from Project Formation's capability requirements and Decision Closure's material elements.

## Run

```sh
rustup toolchain install 1.96.0 --profile minimal
rustup component add rustfmt clippy --toolchain 1.96.0
cargo +1.96.0 generate-lockfile --manifest-path Cargo.toml
cargo +1.96.0 fmt --manifest-path Cargo.toml --all -- --check
cargo +1.96.0 test --manifest-path Cargo.toml --locked
cargo +1.96.0 clippy --manifest-path Cargo.toml --all-targets --locked -- -D warnings
```

## Contract

- The plan digest commits to the full canonical action set, not only a count.
- Action IDs, prerequisite graph and evidence profiles are validated before a plan is sealed.
- Observations bind to the exact plan digest, episode, zone, jurisdiction, context and action ID.
- Duplicate observation IDs are idempotent only for identical canonical content; conflicting duplicates fail as Conflict.
- Multiple observations from one independence group count once.
- Expired, revoked, disputed, attempted, incomplete, stale, future, wrong-plan, wrong-episode and wrong-context observations cannot increase coverage.
- Prerequisites must be covered before a dependent action can count.
- A denominator amendment requires a new sequential revision, exact supersedes digest and a new authority decision digest. Old plans remain independently replayable.
- Reports carry numerator, denominator, basis points, exact covered/uncovered action IDs and rejected-observation reasons.

## Current tests

The unit suite includes canonical ordering, six-authorized/five-observed arithmetic, duplicate idempotence, conflicting identity, common-cause de-duplication, wrong-zone rejection, expired/revoked evidence, prerequisite gating, authority-bound amendment/historical replay, same-count/different-set digest distinction and a SHA-256 known-answer test.

## Claim ceiling

A successful test run establishes deterministic accounting over caller-supplied, typed plans and observations. It does not authenticate the issuing authority, observation sources, permit/qualification revocation, field completion, independent professional approval, physical water restoration, or permission to close an incident. The plan must be created only after an upstream owner has actually ratified its authority decision; this crate checks the binding fields and revision transition, not the legal/organizational truth of that authority.
