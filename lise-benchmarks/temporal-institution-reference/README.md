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

## Conformance suite

The synthetic trace suite contains 12 scenarios with a separately maintained oracle:
- normal positive control;
- founder unavailable with high-impact capability still enabled;
- safe degraded read-only recovery;
- degraded service without old-session fencing;
- stale administrator restoration after constitution change;
- high-impact recovery without quorum;
- high-impact recovery after quorum, fencing and credential revalidation;
- expired emergency authority;
- coalition overlap across appointment, treasury, credential and oversight;
- restoration of independent oversight;
- incomplete control inventory;
- recovery without credential revalidation.

The trace and oracle files are under `fixtures/`; `schema.json` describes both formats. The integration test checks every expected disposition and reason code.

## Bounded exploration

`bounded_explore(max_depth, max_states)` performs breadth-first search over the declared finite transition alphabet. It reports:
- number of visited states;
- maximum depth;
- shortest counterexample trace per hard-invariant reason;
- challenge witnesses;
- indeterminate witnesses;
- whether the state budget truncated the search.

The transition alphabet intentionally permits unsafe actions such as restoring old administrator privileges or enabling high-impact capabilities. The checker must discover and report those traces rather than silently prevent them in the model.

## Run

```sh
cargo fmt --check
cargo check --all-targets
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

**11 unit and 2 integration tests are authored. They have not been compiled or executed in this authoring environment.** Exact-head GitHub Actions is the validation authority.

## Non-goals and limitations

This is a bounded research model, not a real institution inventory, authority registry, control-plane interceptor, legal system or formal proof of an actual organization. The capture model simplifies controller identity to one modeled coalition and selected control domains. Real deployments require owner-qualified control edges, time intervals, delegation scope, organization identity and dependency evidence.

The control mask is a synthetic hyperedge approximation, not a complete institutional power ontology. The current temporal checks cover a deliberately small set of properties and finite transitions. Absence of a counterexample within the explored bound is not a proof of universal safety.

No production security, institutional legitimacy, external audit or field effectiveness is claimed.
