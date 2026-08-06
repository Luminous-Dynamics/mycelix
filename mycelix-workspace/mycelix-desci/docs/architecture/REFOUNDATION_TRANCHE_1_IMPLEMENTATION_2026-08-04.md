# Refoundation Tranche 1 Implementation

**Status:** implemented; authority and persistence deferrals are addressed in
[Refoundation Tranche 2](REFOUNDATION_TRANCHE_2_IMPLEMENTATION_2026-08-04.md).

## Included

- Signed, append-only scientific events using Ed25519 and BLAKE3 chaining.
- Separate research objects, atomic claims, evidence artifacts, and attestations.
- Optimistic concurrency in the in-memory scientific event log.
- Deterministic claim projections and multi-dimensional evidence profiles.
- Terminal supersession and retraction state.
- JWT subject extraction into `AuthenticatedActor`.
- API removal of submitter-controlled `tier`, `creator`, and `verifier`.
- Legacy verification ingress that records material but cannot promote maturity.
- Owner-only mutation for legacy provenance.
- Direct trust-score mutation disabled pending a signed-event policy.
- Fake ZK verification and fabricated archive receipts removed.

## Superseded tranche-1 deferrals

Tranche 2 now supplies actor/key binding, key revocation, scientific action
authorization, unique-source evidence qualification, a versioned assessment
policy, canonical wire bytes, and a durable local reference event log.

## Still deferred

- Production identity resolution from JWT, DID, ORCID, and credentials.
- Canonical command routes and CLI integration.
- Holochain persistence and projection rebuilding.
- Explicit migration of existing `DesciClaim` records.
- Real review-integrity AIR verification and archive receipt adapters.

## Compatibility boundary

The legacy mutable claim model remains available so advanced research modules do
not require a flag-day rewrite. New authoritative workflows must target the
governed canonical event path; legacy API verification data remains explicitly
unassessed.
