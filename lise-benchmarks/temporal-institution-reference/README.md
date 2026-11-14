# LISE Temporal Institution Reference

A small, dependency-light finite-state reference model for temporal institutional capture and authority-preserving recovery.

## Scope

The model treats an institution as a temporal state machine with:
- control domains (appointments, agenda, treasury, credentials, execution and oversight);
- a shared-coalition control mask and explicit control-inventory completeness;
- founder availability, recovery trigger, quorum and old-session fencing;
- service mode (normal, degraded read-only, offline);
- current constitution epoch and credential epoch;
- emergency authority with explicit expiry;
- bounded breadth-first state exploration and shortest counterexample traces.

The reference distinguishes:
- **Deny:** a known temporal safety invariant is violated;
- **ChallengeRequired:** declared oversight shares control with a coalition that also controls a critical power domain; this is a review trigger, not a misconduct verdict;
- **Indeterminate:** the control inventory is incomplete;
- **Permit:** only the finite set of declared properties holds for the supplied state.

## Run

```sh
cargo fmt --check
cargo check --all-targets
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

## Non-goals and limitations

This is a bounded research model, not a real institution inventory, authority registry, control-plane interceptor, legal system or formal proof of an actual organization. The capture model simplifies controller identity to one modeled coalition and selected control domains. Real deployments require owner-qualified control edges, time intervals, delegation scope, organization identity and dependency evidence.

The bounded explorer reports its depth, number of visited states and whether the state budget truncated exploration. Absence of a counterexample within a bounded search is not a proof of universal safety.

No production security, institutional legitimacy, external audit or field effectiveness is claimed.
