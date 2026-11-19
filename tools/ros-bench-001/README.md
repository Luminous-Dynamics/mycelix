# ROS-BENCH-001 — Delegated Authority and Revocation Corpus

**Status:** executable synthetic reference corpus v0.1. No Cedar, SpiceDB or OpenFGA adapter results are claimed.

This package freezes expected authorization dispositions for Mycelix delegated grants and consequential mutations. It does not implement an authorization server. The reference oracle defines the outcome that each matched engine adapter must reproduce.

## Included cases

The 23-case TSV corpus covers direct grants, attenuated child delegation, principal substitution, scope/action/resource/purpose/audience expansion, expiry and not-yet-valid grants, revoked leaf and parent grants, stale cached authorization after revocation, missing parents, delegation cycles, parent digest mismatch, excessive depth, nonce replay, duplicate mutation IDs, stale revocation sources and unavailable authority sources.

The reference decision order is deliberate:

1. If the authoritative revocation/source state is stale or unavailable, return `not_evaluable`; do not infer permission.
2. If required authority is not known/current, return `not_evaluable`.
3. If any known identity, scope, action, resource, purpose, audience, validity, revocation, chain, depth, nonce or idempotency constraint fails, return `deny`.
4. Only complete, current, valid evidence can return `allow`.

## Run

```sh
cargo fmt --manifest-path tools/ros-bench-001/Cargo.toml --all -- --check
cargo test --manifest-path tools/ros-bench-001/Cargo.toml
cargo clippy --manifest-path tools/ros-bench-001/Cargo.toml --all-targets -- -D warnings
```

## Next comparison adapters

- **Cedar:** typed policy schema and request/entity validation. Its schema-validation soundness is a valuable baseline, but the benchmark must also test policy intent and source freshness.
- **SpiceDB / Zanzibar-style:** relationship and graph authorization; compare the same grant graph and consistency/revocation expectations.
- **OpenFGA:** optional relationship-model comparator after a semantics-preserving mapping is available.

Do not rank these systems by raw latency until the same decisions, consistency model, cache state, graph shape, policy/schema, hardware, and freshness requirements are matched. Record engine decision, source/relationship reads, cache/consistency parameters, P50/P95/P99, throughput and revocation visibility window separately.

## Claim ceiling

A pass means the reference implementation matches this synthetic expected-disposition corpus. It does not establish legal authority, identity assurance, real revocation freshness, Holochain DHT behavior, or superiority over an external authorization engine. External-engine differential runs are a separate phase.
