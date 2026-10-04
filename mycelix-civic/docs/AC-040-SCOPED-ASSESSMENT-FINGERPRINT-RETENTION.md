# AC-040 — Scoped Assessment Fingerprint Retention

## Purpose

AC-036 binds lifecycle revisions to an exact scope fingerprint.

AC-040 carries the same fingerprint into the scoped integrity assessment itself.

This prevents downstream systems from retaining only:

`action_ref + scope_id + decision`

while losing the exact scope contents that produced the decision.

## Rule

`EconomicIntegrityGate::assess_scoped` now returns:

- action reference;
- scope ID;
- exact scope fingerprint;
- known covered impact IDs;
- AC-025 composed integrity decision.

The fingerprint is computed from the complete AC-030 scope using the versioned canonicalization scheme defined in AC-036.

## Audit value

A downstream payment, approval, or reporting record can now persist the assessment fingerprint directly.

An auditor can therefore compare:

`assessment.scope_fingerprint`

against the lifecycle's active fingerprint without reconstructing the scope from mutable external state.

## Invariants

- scope identity remains paired with scope content identity;
- changing any material scope field changes the fingerprint;
- the assessment cannot silently detach from the scope it evaluated;
- existing fail-closed substrate/impact composition is unchanged.

## Validation

Added regression coverage proves a scoped assessment returns exactly the fingerprint calculated from the evaluated scope.

## Architectural result

The authorization chain is now content-bound at three places:

`scope -> lifecycle revision -> integrity assessment -> execution receipt`

This creates a continuous evidence anchor from policy declaration through execution.
