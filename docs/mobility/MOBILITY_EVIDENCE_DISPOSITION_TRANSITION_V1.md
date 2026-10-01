# Mobility evidence disposition transition semantics v1

This document defines the append-only transition layer for epistemic disposition.

## Separate dimensions

A mobility evidence record carries three independent dimensions:

1. **Event interval** — when the evidence-generating event occurred.
2. **Effectivity interval** — when the evidence was asserted applicable to the exact configuration/artifact binding.
3. **Disposition** — the currently documented epistemic state of that evidence.

A disposition transition never edits either temporal interval.

## Transition rules

- The initial disposition transition starts from `Active`.
- Every later transition names exactly one explicit predecessor.
- Every transition names an addressable typed basis witness.
- `Active` may transition to `Disputed`, `Superseded`, `Retracted`, or `Unresolved`.
- `Disputed` may resolve to `Active`, or transition to `Superseded`, `Retracted`, or `Unresolved`.
- `Unresolved` may transition to `Active`, `Disputed`, `Superseded`, or `Retracted`.
- `Superseded` and `Retracted` are terminal.
- Same-state transitions are rejected.
- Self-referential predecessors and untyped witnesses are rejected.

A missing predecessor or other external dependency is a dependency-resolution problem at the protocol layer; it must not be silently interpreted as a negative epistemic finding.

## Concurrency

Two independently authored transitions may describe competing branches from the same predecessor. This model does not silently collapse those branches. Reconciliation must itself be explicit and addressable.

## Scope boundaries

These transitions do not establish truth, safety, certification, regulatory approval, causal correctness, or engineering authority. They encode provenance and documented epistemic state only.

The deterministic dependency model is intentionally compatible with Holochain validation: addressable dependencies can be validated deterministically, while unavailable dependencies remain unresolved. Holochain's source-chain model is itself append-only and explicitly links records through prior history.

For lifecycle traceability, this is consistent with NIST work describing digital threads as explicit associations across lifecycle stages and temporal alignment of execution data.
