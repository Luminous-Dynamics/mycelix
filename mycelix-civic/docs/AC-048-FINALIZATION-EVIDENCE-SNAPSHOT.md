# AC-048 — Finalization Evidence Snapshot

## Purpose

AC-048 prevents replay of a stale economic finalization assessment.

A `Ready` result is an assessment of a particular evidence state. It must not
be treated as timeless authorization after the underlying evidence changes.

The assessment therefore carries a deterministic SHA-256 fingerprint of the
exact evidence snapshot used to produce it.

## Freshness invariant

An assessment is fresh only when re-evaluating the same action against current
evidence produces:

- the same action reference;
- the same current lifecycle revision;
- the same active scope identity and fingerprint;
- the same evidence snapshot fingerprint;
- the same finalization decision.

The reference API exposes this through
`EconomicFinalizationAssessment::verify_freshness`.

The method deliberately re-runs the gate. Freshness is not inferred from the
old `Ready` bit.

## Snapshot scope

The fingerprint includes:

- the complete lifecycle history;
- the complete active scope;
- current substrate accounts for required dimensions;
- append-only substrate events for required dimensions;
- action-local impact records;
- reconciliation records relevant to the supplied constraints;
- execution receipts referenced by those reconciliations;
- all supplied required constraints in canonical constraint-ID order.

This creates a useful boundary:

**shared ledgers remain globally visible, while action finalization fingerprints
only the evidence that can affect that action.**

An unrelated action can add an impact without invalidating another action's
finalization freshness.

## Why lifecycle history is included

The lifecycle is append-only and historical revisions are meaningful. A fresh
finalization certificate must therefore bind to the whole lifecycle history,
not merely repeat the current revision identifier.

This prevents history mutation or replacement from being treated as equivalent
to the state that was originally assessed.

## Why substrate events are included

The current account value is not the whole evidence story. A sequence of
append-only observations can change the provenance history even when aggregate
state eventually returns to the same value.

Required-dimension substrate events therefore participate in the snapshot
fingerprint.

## Canonicalization

The reference implementation uses serde JSON serialization plus a
domain/version prefix before SHA-256.

Constraint ordering is explicitly canonicalized by `constraint_id`.

The representation should not be described as a universal cross-language
canonical JSON protocol. It is a deterministic reference encoding. A future
interoperability profile may define a stronger canonical format while retaining
the same freshness invariant.

## Relationship to prior controls

AC-045 created the finalization gate.

AC-046 bound execution reconciliations to exact receipt and constraint content
and recomputed conformance semantics.

AC-047 separated action-local impact scope from global impact visibility.

AC-048 adds temporal validity to the resulting certificate:

**a finalization assessment is evidence-bound and state-bound, not indefinitely
replayable.**

## Non-goals

AC-048 does not:

- create an expiration period by arbitrary policy;
- replace signatures or governance authority;
- claim that the underlying evidence is truthful merely because its fingerprint
  is stable;
- eliminate the need for re-assessment when the relevant evidence changes.

## Tests

The reference tests cover:

- an unchanged assessment remaining fresh;
- required substrate changes invalidating freshness;
- unrelated action impacts not invalidating freshness.

