# ADR-0006: Legacy Proof-of-Learning Trust Quarantine

- Status: Proposed
- Subject: `PRAX-POL-001B`
- Dependency: `PRAX-POL-001A` / PR #2516

## Context

The historical `proof_of_learning.rs` implementation predates the evidence-authority model introduced by ADR-0002 and ADR-0005. It combines heterogeneous learning heuristics into a single scalar and exposes `PoLMATLScore::combine`, which can arithmetically adjust a MATL trust score from that scalar.

The historical implementation is useful for reproducibility and migration, but its evidence does not carry the stable provenance-complete event lineage required by the new learning-analysis contract. Its error-pattern and transfer heuristics also do not independently establish learner authenticity, cheating, general trustworthiness, mastery, credential eligibility, or authorization.

## Decision

Praxis keeps the historical PoL algorithm intact for reproducibility and introduces an explicit quarantine boundary around its outputs.

```text
legacy PoL score
!= authenticity proof
!= cheating proof
!= trust evidence
!= credential evidence
!= authorization
```

### Compatibility, not authority

`LegacyPoLQuarantineReceipt` wraps a historical `ProofOfLearning` through the provenance-incomplete `LegacyPoLCompatibilitySummary` introduced by ADR-0005.

It records explicit quarantine reasons and always reports:

```text
evidence_lineage_complete == false
grants_trust_authority() == false
grants_credential_authority() == false
grants_authorization() == false
```

There is intentionally no conversion from this receipt into a provenance-complete `LearningAnalysisProjection`.

### Historical PoL -> MATL arithmetic

`LegacyPoLMATLCompatibilityReceipt::reproduce_historical` may reproduce the historical `PoLMATLScore::combine` arithmetic for regression comparison, migration, and display.

That compatibility receipt records:

```text
trust_policy_present = false
evidence_lineage_complete = false
```

and grants no trust, credential, or authorization authority.

The compatibility function validates that MATL base, legacy PoL score/confidence, and PoL weight are finite normalized values before reproducing the old arithmetic. This validation does not make the result authoritative.

## Why preserve the old algorithm

Changing the historical PoL equations and the authority model in the same tranche would destroy the ability to distinguish:

- semantic-authority repair;
- numerical/algorithmic change;
- migration drift.

Therefore this ADR freezes the old arithmetic as historical behavior while denying it new authority.

## New authoritative direction

New learning analysis should use `LearningAnalysisProjection` from ADR-0005, with exact stable evidence-event lineage and component-level references.

Any future use of learning analysis in a trust decision requires a separate named/versioned trust policy that specifies at least:

- accepted analysis component kinds;
- required evidence basis;
- analyzer and parameter allowlists;
- calibration/validation evidence;
- domain of applicability;
- weight/limit semantics;
- correction/appeal/revocation behavior for consequential use;
- an exact policy receipt over the consumed analysis projections.

No such policy is created by this ADR.

## Compatibility boundary

This tranche does not delete `ProofOfLearning`, `PoLComponents`, `PoLAnalyzer`, or `PoLMATLScore::combine`. Existing callers can therefore remain source-compatible while migration proceeds.

However, new Praxis authority-bearing code must not treat those legacy types as evidence of trust, mastery, authenticity, cheating, credential eligibility, or authorization.

## Tests

The quarantine contract proves that:

1. historical PoL always produces a non-authoritative quarantine receipt;
2. historical PoL->MATL arithmetic can be reproduced exactly through the compatibility wrapper;
3. the reproduced value still has no trust/credential/authorization authority;
4. invalid MATL bases and weights cannot cross the compatibility boundary;
5. evidence lineage remains explicitly incomplete.

## Core theorem

```text
reproducible historical arithmetic
!= validated trust policy

legacy compatibility
!= authority
```
