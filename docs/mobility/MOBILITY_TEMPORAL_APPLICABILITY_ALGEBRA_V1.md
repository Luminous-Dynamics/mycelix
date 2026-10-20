# Mobility Temporal Applicability Algebra v1

## Purpose

Temporal semantics are a separate layer from identity, configuration scope, evidence state, and protocol publication. This contract is semantic/provenance-only.

## Temporal concepts

- **event time**: when an event is claimed to have occurred;
- **applicability interval**: when an assertion/configuration is semantically applicable;
- **publication time**: when the record was published/committed;
- **external-authority time**: time attributed to an external authority;
- **protocol publication time**: protocol metadata, not engineering applicability.

These values must never be silently collapsed.

## Representation

Temporal points are normalized to integer Unix seconds UTC. Conversion from wall-clock text is outside this algebra and must require an unambiguous offset-aware input. A missing start or end is explicitly unknown, not an invented bound.

Intervals are valid only when `end >= start`. Unknown bounds validate structurally but overlap is `indeterminate` rather than guessed.

## Invariants

1. Holochain action timestamps are not engineering validity intervals.
2. Publication time is not event time.
3. Event time does not establish truth.
4. An open-ended interval does not imply current safety.
5. Supersession preserves predecessor temporal history.
6. Retirement preserves historical identity.
7. Temporal overlap does not imply physical equivalence.
8. Later evidence does not erase earlier evidence.
9. Unknown temporal bounds remain unresolved/indeterminate.
10. External authority time remains attributed to that authority.

## Integration

Identity/lineage (#3735) answers which engineering entity is involved. Configuration scope (#3727) answers which configuration context a relationship belongs to. Outcome algebra (#3725) describes epistemic/lifecycle/conflict/authority state. This algebra answers when an assertion or event is claimed to apply.

No temporal result is a safety, certification, regulatory, or engineering-correctness determination.
