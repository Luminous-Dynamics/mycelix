# FSRE decision episode E0 composer

A dependency-free Rust composer for a typed, scoped graph of decision-assurance claims.

## Purpose

Individual subsystems can each be internally correct while referring to incompatible populations, service boundaries, asset namespaces, as-of cutoffs or source snapshots. The composer checks the consistency of the declared decision episode and its proof-obligation graph.

## E0 behavior

- Closed vocabulary of required claim types and qualification states.
- Exact scope-ID and canonical scope-digest match between every claim and the episode.
- Strict SHA-256 reference shape (64 lowercase hexadecimal characters) for the scope, artifacts and source snapshots.
- Conflicting digest references for a reused artifact ID are rejected.
- Required claim coverage, duplicate claim/edge checks, dangling-reference rejection and acyclic dependency graph.
- Fail-closed terminal states for blocked, unknown, missing and scope-inconsistent claims.
- Deterministic receipt independent of semantically irrelevant input row ordering.

## What it does not do

The composer does not recompute the canonical scope digest or rehash artifact bytes, authenticate source origin, establish legal authority, validate the truth of subsystem claims, prove the adequacy of a world set, certify physical realizability, or authorize action. It records that an independent-verifier claim was supplied, but does not independently re-authenticate that verifier receipt. A terminal status of DECISION_ASSURANCE_COMPLETE_WITHIN_SCOPE means only that the declared claim set is complete and internally consistent within the supplied scope.

## Run

    cargo fmt --manifest-path tools/fsre-decision-episode/Cargo.toml -- --check
    cargo test --locked --offline --manifest-path tools/fsre-decision-episode/Cargo.toml
    cargo clippy --locked --offline --manifest-path tools/fsre-decision-episode/Cargo.toml --all-targets -- -D warnings
    cargo run --quiet --locked --offline --manifest-path tools/fsre-decision-episode/Cargo.toml -- tools/fsre-decision-episode/fixtures/complete-episode.psv

## Claim ceiling

The composer never grants intervention authority. It emits decision_authorization=NOT_GRANTED_BY_COMPOSER in every report. Content hashing and independent receipt verification must be performed by their respective verifier components.
