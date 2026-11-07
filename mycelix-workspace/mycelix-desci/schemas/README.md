# Black-hole evidence package

This directory defines a domain-specific manifest for black-hole physics research. It is deliberately layered on top of Mycelix-DeSci's existing append-only scientific-event kernel.

## Ownership boundary

The manifest does **not** create a second claim ledger or second evidence maturity system:

- Mycelix's `ResearchObject`, `AtomicClaim`, `EvidenceArtifact`, `Attestation`, `ClaimLifecycle`, and signed `ScientificEventEnvelope` remain canonical.
- The package references the existing `ClaimId`, `ResearchObjectId`, `ArtifactId`, and event IDs.
- Evidence maturity and whether a claim is contested continue to be derived by the existing versioned evidence policy from event history.
- The package supplies black-hole-specific scientific metadata: equations, assumptions, units, coordinate chart, implementation SHAs, input/output artifact digests, uncertainty categories, observables, null controls, and verification outcomes.

## Integrity and scientific validity

The `artifacts[].digest` field uses BLAKE3-256 to match `ContentHash` in the existing event kernel. It hashes the exact stored artifact bytes; this schema does not impose a second canonical-JSON serialization rule.

A digest establishes byte identity/integrity, not scientific correctness. A signed event establishes who attested to a record, not that the physical interpretation is true. Independent replication, null controls, and cross-method checks remain separately typed evidence.

## Epistemic separation

The manifest's `claim.epistemic_kind` distinguishes theory, derivation, observation, inference, simulation, hypothesis, and disputed interpretation. Mycelix event lifecycle remains authoritative for active, superseded, and retracted states. Critiques and non-supporting replications must be preserved as events; do not overwrite the package or resolve scientific disagreement by vote.

## Uncertainty

Do not collapse uncertainty into one confidence score. Record statistical, systematic, numerical truncation, discretization, model-form, calibration, noise, astrophysical population, parameter-degeneracy, and surrogate-model uncertainty separately, using the appropriate representation and linked artifact.

## Validation

Schema version 1 uses JSON Schema Draft 2020-12. Cross-object rules such as artifact IDs resolving to actual event artifacts, event stream ownership, source/version pinning, and whether an independent verifier is truly independent must be enforced by a higher-level package validator and the signed Mycelix event path; JSON Schema alone cannot establish these facts.
